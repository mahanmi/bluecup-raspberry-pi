import asyncio
import logging


class CustomFormatter(logging.Formatter):
    grey = "\x1b[38;20m"
    yellow = "\x1b[33;20m"
    red = "\x1b[31;20m"
    bold_red = "\x1b[31;1m"
    reset = "\x1b[0m"
    _format = "%(asctime)s - %(name)s - %(levelname)s - %(message)s (%(filename)s:%(lineno)d)"

    FORMATS = {
        logging.DEBUG: grey + _format + reset,
        logging.INFO: grey + _format + reset,
        logging.WARNING: yellow + _format + reset,
        logging.ERROR: red + _format + reset,
        logging.CRITICAL: bold_red + _format + reset
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)


class CustomLogger(logging.Logger):
    def __init__(self, name, level=logging.INFO):
        super().__init__(name, level)
        handler = logging.StreamHandler()
        handler.setFormatter(CustomFormatter())
        self.addHandler(handler)


logging.setLoggerClass(CustomLogger)


async def main():
    from api import AsyncMessageThread
    message_thread = AsyncMessageThread()

    try:
        # Start the background tasks. This method returns immediately.
        await message_thread.start()

        # This is the crucial part. We wait on a future that never
        # completes, keeping the program alive until we interrupt it.
        print("--- Program running. Press Ctrl+C to stop. ---")
        await asyncio.Future()

    except asyncio.CancelledError:
        print("\nAsyncio task was cancelled.")
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received.")
    finally:
        print("Shutting down...")
        # Ensure a graceful shutdown by calling your stop method
        message_thread.stop()
        # Give a moment for cancellation messages to be processed
        await asyncio.sleep(0.1)
        print("Shutdown complete.")

if __name__ == "__main__":
    asyncio.run(main())
