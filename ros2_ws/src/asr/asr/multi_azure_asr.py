import threading
import time
import uuid
from queue import Empty, Queue

import azure.cognitiveservices.speech as speechsdk
import rclpy
import requests
from rclpy.node import Node
from std_msgs.msg import String

GREEN = '\033[92m'
RESET = '\033[0m'

# Maps Azure locale codes to English language names used in the output marker.
# Extend this dict when adding languages to the `languages` parameter.
LANGUAGE_NAME_MAP = {
    'ja-JP': 'Japanese',
}

_AZURE_TRANSLATOR_ENDPOINT = 'https://api.cognitive.microsofttranslator.com'


class MultiAzureASR(Node):
    """Multilingual Azure Speech Recognition ROS 2 Node.

    Extends the single-language ASR concept to N preset languages using Azure's
    AutoDetectSourceLanguageConfig. Recognized speech that is not in English is
    automatically translated to English via the Azure Translator REST API and
    published with a language marker, e.g. "Hello <Japanese>".

    Features:
    - Continuous multilingual speech recognition from microphone
    - Automatic language detection across N configured languages
    - On-the-fly translation of non-English speech to English
    - Language marker appended to translated output: "<LanguageName>"
    - Thread-safe message publishing via a Queue drained by a ROS timer
    - Graceful shutdown handling
    - Configurable via ROS 2 parameters
    """

    def __init__(self, **kwargs):
        super().__init__('multi_azure_asr', **kwargs)

        # Declare ROS 2 parameters
        self.declare_parameter('asr_topic', '/asr')
        self.declare_parameter('language_topic', '/language')
        self.declare_parameter('azure_speech_key', '')
        self.declare_parameter('azure_speech_region', '')
        self.declare_parameter('enable_semantic_segmentation', True)
        self.declare_parameter('languages', ['en-US', 'ja-JP'])
        self.declare_parameter('azure_translate_key', '')
        self.declare_parameter('azure_translate_region', '')
        self.declare_parameter('translate_to_eng', False)

        # Get parameter values
        self.asr_topic_name = self.get_parameter('asr_topic').value
        self.language_topic_name = self.get_parameter('language_topic').value
        self.azure_speech_key = self.get_parameter('azure_speech_key').value
        self.azure_speech_region = self.get_parameter(
            'azure_speech_region').value
        self.enable_semantic_segmentation = self.get_parameter(
            'enable_semantic_segmentation').value
        self.languages = self.get_parameter('languages').value
        self.azure_translate_key = self.get_parameter(
            'azure_translate_key').value
        self.azure_translate_region = self.get_parameter(
            'azure_translate_region').value
        self.translate_to_eng = self.get_parameter('translate_to_eng').value

        # Create publisher
        self.asr_publisher = self.create_publisher(
            String,
            self.asr_topic_name,
            10,
        )

        # Threading components
        self._recognition_thread = None
        self._shutdown_event = threading.Event()
        self._message_queue = Queue()

        # Azure Speech SDK components
        self._speech_recognizer = None

        # Initialize Azure Speech Recognition
        self._initialize_speech_recognition()

        # Start background threads
        self._start_background_threads()

        self.get_logger().info(
            'MultiAzureASR initialized\n'
            'Parameters:\n'
            f'  asr_topic_name: {self.asr_topic_name}\n'
            f'  language_topic_name: {self.language_topic_name}\n'
            f'  azure_speech_key: {self.azure_speech_key[:8]}...\n'
            f'  azure_speech_region: {self.azure_speech_region}\n'
            f'  enable_semantic_segmentation: {self.enable_semantic_segmentation}\n'  # noqa
            f'  languages: {self.languages}\n'
            f'  azure_translate_key: {self.azure_translate_key[:8]}...\n'
            f'  azure_translate_region: {self.azure_translate_region}')

    def _initialize_speech_recognition(self):
        """Initialize Azure Speech Recognition with multilingual auto-detection."""
        try:
            if not self.azure_speech_key or not self.azure_speech_region:
                raise ValueError(
                    'Azure Speech credentials not found. Set azure_speech_key '
                    'and azure_speech_region parameters')

            if not self.languages:
                raise ValueError(
                    'No languages configured. Set the languages parameter to '
                    'a non-empty list of locale codes (e.g. ["en-US","ja-JP"])'
                )

            speech_config = speechsdk.SpeechConfig(
                subscription=self.azure_speech_key,
                region=self.azure_speech_region,
            )

            # Enable continuous language identification
            speech_config.set_property(
                property_id=speechsdk.PropertyId.
                SpeechServiceConnection_LanguageIdMode,
                value='Continuous',
            )

            # Enable semantic segmentation if configured
            if self.enable_semantic_segmentation:
                speech_config.set_property(
                    speechsdk.PropertyId.Speech_SegmentationStrategy,
                    'Semantic')

            # Configure automatic language detection across all preset languages
            auto_detect_config = (
                speechsdk.languageconfig.AutoDetectSourceLanguageConfig(
                    languages=list(self.languages), ))

            # Create speech recognizer with language auto-detection
            self._speech_recognizer = speechsdk.SpeechRecognizer(
                speech_config=speech_config,
                auto_detect_source_language_config=auto_detect_config,
            )

            self._setup_recognition_handlers()

            self.get_logger().info('Azure Speech Recognizer initialized')

        except Exception as e:
            self.get_logger().error(
                f'Failed to initialize Azure Speech Recognizer: {e}')
            raise

    def _setup_recognition_handlers(self):
        """Set up event handlers for Azure Speech Recognition events."""

        def on_recognized(evt):
            if evt.result.reason == speechsdk.ResultReason.RecognizedSpeech:
                text = evt.result.text.strip()
                if not text:
                    return

                if self.translate_to_eng:
                    auto_detect_result = speechsdk.AutoDetectSourceLanguageResult(
                        evt.result)
                    lang = auto_detect_result.language  # e.g. "ja-JP"

                    if lang and not lang.startswith('en'):
                        lang_name = LANGUAGE_NAME_MAP.get(lang, lang)
                        try:
                            translated = self._translate_to_english(text, lang)
                            output = f'{translated} <{lang_name}>'
                        except Exception as e:
                            self.get_logger().error(
                                f'Translation failed for "{text}" ({lang}): {e}'
                            )
                            output = f'{text} <{lang_name}>'
                    else:
                        output = text
                else:
                    output = text

                self._message_queue.put(output)
                self.get_logger().info(f'{GREEN}Recognized: {output}{RESET}')

            elif evt.result.reason == speechsdk.ResultReason.NoMatch:
                self.get_logger().debug('No speech could be recognized')

        def on_canceled(evt):
            if evt.reason == speechsdk.CancellationReason.Error:
                self.get_logger().error(
                    f'Speech recognition canceled due to error: '
                    f'{evt.error_details}')
            else:
                self.get_logger().info('Speech recognition canceled')

        def on_session_started(evt):
            self.get_logger().info('Speech recognition session started')

        def on_session_stopped(evt):
            self.get_logger().info('Speech recognition session stopped')

        self._speech_recognizer.recognized.connect(on_recognized)
        self._speech_recognizer.canceled.connect(on_canceled)
        self._speech_recognizer.session_started.connect(on_session_started)
        self._speech_recognizer.session_stopped.connect(on_session_stopped)

    def _translate_to_english(self, text: str, from_lang: str) -> str:
        """Translate text from the detected language to English.

        Uses the Azure Translator REST API (v3). The locale code is trimmed to
        its language prefix (e.g. "ja-JP" → "ja") as required by the API.

        Args:
            text: The recognized text to translate.
            from_lang: The detected locale code (e.g. "ja-JP").

        Returns:
            The translated English text.

        Raises:
            requests.HTTPError: On a non-2xx HTTP response.
            KeyError/IndexError: If the response shape is unexpected.
        """
        lang_prefix = from_lang.split('-')[0]  # "ja-JP" → "ja"

        url = f'{_AZURE_TRANSLATOR_ENDPOINT}/translate'
        params = {
            'api-version': '3.0',
            'from': lang_prefix,
            'to': 'en',
        }
        headers = {
            'Ocp-Apim-Subscription-Key': self.azure_translate_key,
            'Ocp-Apim-Subscription-Region': self.azure_translate_region,
            'Content-Type': 'application/json',
            'X-ClientTraceId': str(uuid.uuid4()),
        }
        body = [{'text': text}]

        response = requests.post(url,
                                 params=params,
                                 headers=headers,
                                 json=body,
                                 timeout=10)
        response.raise_for_status()

        return response.json()[0]['translations'][0]['text']

    def _start_background_threads(self):
        """Start background threads for recognition and message publishing."""
        self._recognition_thread = threading.Thread(
            target=self._recognition_worker,
            name='MultiAzureASR_Recognition',
            daemon=True)
        self._recognition_thread.start()

        self._publisher_timer = self.create_timer(0.1, self._publish_messages)

    def _recognition_worker(self):
        """Background worker thread for continuous speech recognition."""
        try:
            self.get_logger().info('Starting continuous speech recognition...')
            self._speech_recognizer.start_continuous_recognition()

            while not self._shutdown_event.is_set():
                time.sleep(0.5)

        except Exception as e:
            self.get_logger().error(f'Recognition worker error: {e}')
        finally:
            if self._speech_recognizer:
                try:
                    self._speech_recognizer.stop_continuous_recognition()
                    self.get_logger().info(
                        'Continuous speech recognition stopped')
                except Exception as e:
                    self.get_logger().error(f'Error stopping recognition: {e}')

    def _publish_messages(self):
        """Timer callback to publish queued recognition messages."""
        try:
            while not self._message_queue.empty():
                try:
                    text = self._message_queue.get_nowait()

                    msg = String()
                    msg.data = text
                    self.asr_publisher.publish(msg)

                    self._message_queue.task_done()

                except Empty:
                    break
                except Exception as e:
                    self.get_logger().error(f'Error publishing message: {e}')

        except Exception as e:
            self.get_logger().error(f'Publisher timer error: {e}')

    def destroy_node(self):
        """Clean up resources when node is destroyed."""
        self.get_logger().info('Shutting down MultiAzureASR node...')

        self._shutdown_event.set()

        if self._recognition_thread and self._recognition_thread.is_alive():
            self._recognition_thread.join(timeout=5.0)
            if self._recognition_thread.is_alive():
                self.get_logger().warning(
                    'Recognition thread did not stop gracefully')

        if self._speech_recognizer:
            try:
                self._speech_recognizer.stop_continuous_recognition()
            except Exception as e:
                self.get_logger().error(
                    f'Error during recognition cleanup: {e}')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = MultiAzureASR()
        node.get_logger().info('MultiAzureASR node is running...')
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error running MultiAzureASR node: {e}')
    finally:
        try:
            if 'node' in locals():
                node.destroy_node()
        except Exception as e:
            print(f'Error during node cleanup: {e}')
        finally:
            rclpy.shutdown()


if __name__ == '__main__':
    main()
