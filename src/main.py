from api import AsyncMessageThread
import asyncio


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
