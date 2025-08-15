from api import AsyncMessageThread
import asyncio
import logging

# Configure logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


async def main():
    message_thread = AsyncMessageThread()

    try:
        # Start the background tasks. This method returns immediately.
        await message_thread.start()

        # This is the crucial part. We wait on a future that never
        # completes, keeping the program alive until we interrupt it.
        print("--- Program running. Press Ctrl+C to stop. ---")
        await asyncio.Future()

    except KeyboardInterrupt:
        logger.debug("\nKeyboard interrupt received.")
    finally:
        logger.info("Shutting down...")
        # Ensure a graceful shutdown by calling your stop method
        message_thread.stop()
        # Give a moment for cancellation messages to be processed
        await asyncio.sleep(0.1)
        logger.info("Shutdown complete.")

if __name__ == "__main__":
    asyncio.run(main())
