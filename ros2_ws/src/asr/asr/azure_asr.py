import os
import threading
import time
from queue import Empty, Queue

import azure.cognitiveservices.speech as speechsdk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AzureASR(Node):
    """Azure Speech Recognition ROS 2 Node with Continuous Recognition

    This node provides continuous speech recognition using Azure Cognitive
    Services Speech SDK. It runs continuous recognition in a background thread
    and publishes recognized speech text to a ROS 2 topic with low latency.

    Features:
    - Continuous speech recognition from microphone
    - Thread-safe message publishing
    - Graceful shutdown handling
    - Configurable Azure credentials via ROS parameters
    - English language recognition (expandable)
    """

    def __init__(self, **kwargs):
        """Initialize the AzureASR node with speech recognition capabilities."""
        super().__init__('azure_asr', **kwargs)

        # Declare ROS 2 parameters
        self.declare_parameter('asr_topic', '/asr')
        self.declare_parameter('language_topic', '/language')
        self.declare_parameter('azure_speech_key', '')
        self.declare_parameter('azure_speech_region', '')
        self.declare_parameter('enable_semantic_segmentation', True)
        # Get parameter values
        self.asr_topic_name = self.get_parameter('asr_topic').value
        self.language_topic_name = self.get_parameter('language_topic').value
        self.azure_speech_key = self.get_parameter('azure_speech_key').value
        self.azure_speech_region = self.get_parameter(
            'azure_speech_region').value
        # TODO What is this??
        self.enable_semantic_segmentation = self.get_parameter(
            'enable_semantic_segmentation').value

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

        self.get_logger().info('Azure ASR node initialized successfully')

        self.get_logger().info(
            'AzureASR initialized\n'
            'Parameters:\n'
            f'  asr_topic_name: {self.asr_topic_name}\n'
            f'  language_topic_name: {self.language_topic_name}\n'
            f'  azure_speech_key: {self.azure_speech_key[:8]}...\n'
            f'  azure_speech_region: {self.azure_speech_region}\n'
            f'  enable_semantic_segmentation: {self.enable_semantic_segmentation}'
        )

    def _initialize_speech_recognition(self):
        """Initialize Azure Speech Recognition with continuous recognition."""
        try:
            if not self.azure_speech_key or not self.azure_speech_region:
                raise ValueError(
                    'Azure Speech credentials not found. Set azure_speech_key '
                    'and azure_speech_region parameters')

            # Configure speech settings for English recognition
            speech_config = speechsdk.SpeechConfig(
                subscription=self.azure_speech_key,
                region=self.azure_speech_region,
            )

            # Set language to English (US)
            speech_config.speech_recognition_language = "en-US"

            # Enable semantic segmentation if configured
            if self.enable_semantic_segmentation:
                speech_config.set_property(
                    speechsdk.PropertyId.Speech_SegmentationStrategy,
                    "Semantic")

            # Create speech recognizer (uses default microphone)
            self._speech_recognizer = speechsdk.SpeechRecognizer(
                speech_config=speech_config)

            # Set up event handlers
            self._setup_recognition_handlers()

            self.get_logger().info('Azure Speech Recognizer initialized')

        except Exception as e:
            self.get_logger().error(
                f'Failed to initialize Azure Speech Recognizer: {e}')
            raise

    def _setup_recognition_handlers(self):
        """Set up event handlers for Azure Speech Recognition events."""

        def on_recognized(evt):
            """Handle recognized speech results."""
            if evt.result.reason == speechsdk.ResultReason.RecognizedSpeech:
                text = evt.result.text.strip()
                if text:  # Only process non-empty text
                    # Thread-safe message queuing
                    self._message_queue.put(text)
                    self.get_logger().info(f'Recognized: {text}')
            elif evt.result.reason == speechsdk.ResultReason.NoMatch:
                self.get_logger().debug('No speech could be recognized')

        def on_canceled(evt):
            """Handle recognition cancellation."""
            if evt.reason == speechsdk.CancellationReason.Error:
                self.get_logger().error(
                    f'Speech recognition canceled due to error: '
                    f'{evt.error_details}')
            else:
                self.get_logger().info('Speech recognition canceled')

        def on_session_started(evt):
            """Handle recognition session start."""
            self.get_logger().info('Speech recognition session started')

        def on_session_stopped(evt):
            """Handle recognition session stop."""
            self.get_logger().info('Speech recognition session stopped')

        # Connect event handlers
        self._speech_recognizer.recognized.connect(on_recognized)
        self._speech_recognizer.canceled.connect(on_canceled)
        self._speech_recognizer.session_started.connect(on_session_started)
        self._speech_recognizer.session_stopped.connect(on_session_stopped)

    def _start_background_threads(self):
        """Start background threads for recognition and message publishing."""
        # Start recognition thread
        self._recognition_thread = threading.Thread(
            target=self._recognition_worker,
            name='AzureASR_Recognition',
            daemon=True)
        self._recognition_thread.start()

        # Start message publishing timer
        self._publisher_timer = self.create_timer(0.1, self._publish_messages)

    def _recognition_worker(self):
        """Background worker thread for continuous speech recognition."""
        try:
            self.get_logger().info('Starting continuous speech recognition...')

            # Start continuous recognition
            self._speech_recognizer.start_continuous_recognition()

            # Keep thread alive until shutdown
            while not self._shutdown_event.is_set():
                time.sleep(0.5)

        except Exception as e:
            self.get_logger().error(f'Recognition worker error: {e}')
        finally:
            # Stop recognition when shutting down
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
            # Process all queued messages
            while not self._message_queue.empty():
                try:
                    text = self._message_queue.get_nowait()

                    # Create and publish ROS message
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
        self.get_logger().info('Shutting down Azure ASR node...')

        # Signal threads to stop
        self._shutdown_event.set()

        # Wait for recognition thread to finish
        if self._recognition_thread and self._recognition_thread.is_alive():
            self._recognition_thread.join(timeout=5.0)
            if self._recognition_thread.is_alive():
                self.get_logger().warning(
                    'Recognition thread did not stop gracefully')

        # Stop speech recognition
        if self._speech_recognizer:
            try:
                self._speech_recognizer.stop_continuous_recognition()
            except Exception as e:
                self.get_logger().error(
                    f'Error during recognition cleanup: {e}')

        # Call parent cleanup
        super().destroy_node()


def main(args=None):
    """Main entry point for the Azure ASR node.

    Initializes the ROS 2 system, creates an AzureASR instance,
    and runs the node until shutdown. This function handles the complete
    lifecycle of the Azure ASR node with proper cleanup.

    Args:
        args: Command line arguments passed to rclpy.init() (optional)

    Lifecycle:
        1. Initialize ROS 2 system
        2. Create Azure ASR node instance
        3. Log startup message
        4. Enter ROS 2 spin loop to handle callbacks and timers
        5. Clean up resources on shutdown (Ctrl+C or SIGTERM)

    The node will continuously run speech recognition and publish
    recognized text until interrupted or shutdown.
    """
    rclpy.init(args=args)

    try:
        azure_asr_node = AzureASR()
        azure_asr_node.get_logger().info('Azure ASR node is running...')
        rclpy.spin(azure_asr_node)

    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully
    except Exception as e:
        print(f"Error running Azure ASR node: {e}")
    finally:
        # Ensure proper cleanup
        try:
            if 'azure_asr_node' in locals():
                azure_asr_node.destroy_node()
        except Exception as e:
            print(f"Error during node cleanup: {e}")
        finally:
            rclpy.shutdown()


if __name__ == '__main__':
    main()
