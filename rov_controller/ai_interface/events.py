"""Event system for the ROV Controller AI Interface."""

from enum import Enum
from typing import Callable, Dict, List, Any
from dataclasses import dataclass
import asyncio
from concurrent.futures import ThreadPoolExecutor


class EventType(Enum):
    """Types of events that can be subscribed to."""
    SENSOR_UPDATE = "sensor_update"
    CAMERA_FRAME = "camera_frame"
    STATE_CHANGE = "state_change"
    EMERGENCY = "emergency"
    MODE_CHANGE = "mode_change"
    BATTERY_LOW = "battery_low"
    ERROR = "error"
    WARNING = "warning"


@dataclass
class EventHandler:
    """Event handler container."""
    callback: Callable
    async_handler: bool = False


class EventEmitter:
    """Event emitter implementation for the ROV controller."""
    
    def __init__(self):
        """Initialize the event emitter."""
        self._handlers: Dict[EventType, List[EventHandler]] = {
            event_type: [] for event_type in EventType
        }
        self._executor = ThreadPoolExecutor(max_workers=4)
        self._loop = asyncio.get_event_loop()

    def on(self, event_type: EventType, handler: Callable, async_handler: bool = False) -> None:
        """Register an event handler.
        
        Args:
            event_type: Type of event to listen for
            handler: Callback function to handle the event
            async_handler: Whether the handler is an async function
        """
        if event_type not in self._handlers:
            raise ValueError(f"Unknown event type: {event_type}")
        
        self._handlers[event_type].append(EventHandler(handler, async_handler))

    def remove_handler(self, event_type: EventType, handler: Callable) -> None:
        """Remove an event handler.
        
        Args:
            event_type: Type of event to remove handler from
            handler: Handler to remove
        """
        if event_type in self._handlers:
            self._handlers[event_type] = [
                h for h in self._handlers[event_type] 
                if h.callback != handler
            ]

    async def emit(self, event_type: EventType, data: Any = None) -> None:
        """Emit an event to all registered handlers.
        
        Args:
            event_type: Type of event to emit
            data: Data to pass to handlers
        """
        if event_type not in self._handlers:
            raise ValueError(f"Unknown event type: {event_type}")

        for handler in self._handlers[event_type]:
            if handler.async_handler:
                # If handler is async, schedule it directly
                asyncio.create_task(handler.callback(data))
            else:
                # If handler is sync, run it in thread pool
                await self._loop.run_in_executor(
                    self._executor,
                    handler.callback,
                    data
                )

    def emit_sync(self, event_type: EventType, data: Any = None) -> None:
        """Synchronously emit an event to all registered handlers.
        
        This method should be used carefully as it will block until all handlers complete.
        Prefer the async emit() method when possible.
        
        Args:
            event_type: Type of event to emit
            data: Data to pass to handlers
        """
        if event_type not in self._handlers:
            raise ValueError(f"Unknown event type: {event_type}")

        for handler in self._handlers[event_type]:
            if handler.async_handler:
                # Run async handler in new event loop
                loop = asyncio.new_event_loop()
                try:
                    loop.run_until_complete(handler.callback(data))
                finally:
                    loop.close()
            else:
                # Run sync handler directly
                handler.callback(data)

    def clear_handlers(self, event_type: EventType = None) -> None:
        """Clear all handlers for a specific event type or all events.
        
        Args:
            event_type: Type of event to clear handlers for. If None, clears all handlers.
        """
        if event_type is None:
            self._handlers = {event_type: [] for event_type in EventType}
        elif event_type in self._handlers:
            self._handlers[event_type] = [] 