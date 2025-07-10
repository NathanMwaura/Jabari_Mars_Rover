#!/usr/bin/env python3
"""
WebSocket handler for streaming camera data to web clients.
Handles WebSocket connections and manages real-time image streaming.
"""

import asyncio
import websockets
import json
import base64
import logging
from typing import Set, Optional
import threading
import queue
import time

class WebSocketHandler:
    def __init__(self, host: str = "localhost", port: int = 8765):
        """
        Initialize WebSocket handler.
        
        Args:
            host: WebSocket server host
            port: WebSocket server port
        """
        self.host = host
        self.port = port
        self.clients: Set[websockets.WebSocketServerProtocol] = set()
        self.image_queue = queue.Queue(maxsize=10)  # Buffer for images
        self.server = None
        self.loop = None
        self.logger = logging.getLogger(__name__)
        
        # Statistics
        self.frames_sent = 0
        self.last_stats_time = time.time()
        
    async def register_client(self, websocket: websockets.WebSocketServerProtocol):
        """Register a new WebSocket client."""
        self.clients.add(websocket)
        self.logger.info(f"Client connected. Total clients: {len(self.clients)}")
        
        # Send initial connection message
        await self.send_to_client(websocket, {
            "type": "connection",
            "status": "connected",
            "message": "Camera stream ready"
        })
        
    async def unregister_client(self, websocket: websockets.WebSocketServerProtocol):
        """Unregister a WebSocket client."""
        self.clients.discard(websocket)
        self.logger.info(f"Client disconnected. Total clients: {len(self.clients)}")
        
    async def send_to_client(self, websocket: websockets.WebSocketServerProtocol, data: dict):
        """Send data to a specific client."""
        try:
            await websocket.send(json.dumps(data))
        except websockets.exceptions.ConnectionClosed:
            await self.unregister_client(websocket)
        except Exception as e:
            self.logger.error(f"Error sending to client: {e}")
            
    async def broadcast_to_all(self, data: dict):
        """Broadcast data to all connected clients."""
        if not self.clients:
            return
            
        # Create list of tasks for concurrent sending
        tasks = []
        disconnected_clients = set()
        
        for client in self.clients.copy():
            try:
                tasks.append(asyncio.create_task(client.send(json.dumps(data))))
            except websockets.exceptions.ConnectionClosed:
                disconnected_clients.add(client)
            except Exception as e:
                self.logger.error(f"Error preparing broadcast: {e}")
                disconnected_clients.add(client)
        
        # Remove disconnected clients
        for client in disconnected_clients:
            await self.unregister_client(client)
            
        # Wait for all sends to complete
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
            
    async def handle_client_message(self, websocket: websockets.WebSocketServerProtocol, message: str):
        """Handle incoming messages from clients."""
        try:
            data = json.loads(message)
            msg_type = data.get("type", "unknown")
            
            if msg_type == "ping":
                await self.send_to_client(websocket, {"type": "pong"})
            elif msg_type == "request_frame":
                # Client requesting a frame - handled by image streaming
                pass
            elif msg_type == "camera_control":
                # Handle camera control commands
                command = data.get("command")
                self.logger.info(f"Camera control command: {command}")
                # Add camera control logic here if needed
            else:
                self.logger.warning(f"Unknown message type: {msg_type}")
                
        except json.JSONDecodeError:
            self.logger.error("Invalid JSON received from client")
        except Exception as e:
            self.logger.error(f"Error handling client message: {e}")
            
    async def client_handler(self, websocket: websockets.WebSocketServerProtocol, path: str):
        """Handle individual client connections."""
        await self.register_client(websocket)
        
        try:
            async for message in websocket:
                await self.handle_client_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            pass
        except Exception as e:
            self.logger.error(f"Error in client handler: {e}")
        finally:
            await self.unregister_client(websocket)
            
    def add_image(self, image_data: bytes, width: int, height: int, encoding: str = "bgr8"):
        """
        Add image data to the queue for streaming.
        
        Args:
            image_data: Raw image data as bytes
            width: Image width
            height: Image height
            encoding: Image encoding format
        """
        try:
            # Convert image data to base64 for JSON transmission
            base64_image = base64.b64encode(image_data).decode('utf-8')
            
            image_msg = {
                "type": "image_frame",
                "data": base64_image,
                "width": width,
                "height": height,
                "encoding": encoding,
                "timestamp": time.time()
            }
            
            # Add to queue (non-blocking, drop oldest if full)
            if self.image_queue.full():
                try:
                    self.image_queue.get_nowait()  # Remove oldest
                except queue.Empty:
                    pass
                    
            self.image_queue.put_nowait(image_msg)
            
        except Exception as e:
            self.logger.error(f"Error adding image to queue: {e}")
            
    async def image_streamer(self):
        """Continuous image streaming coroutine."""
        while True:
            try:
                # Check if we have clients and images to send
                if not self.clients:
                    await asyncio.sleep(0.1)
                    continue
                    
                # Get image from queue (non-blocking)
                try:
                    image_msg = self.image_queue.get_nowait()
                    await self.broadcast_to_all(image_msg)
                    
                    self.frames_sent += 1
                    
                    # Log statistics every 100 frames
                    if self.frames_sent % 100 == 0:
                        current_time = time.time()
                        fps = 100 / (current_time - self.last_stats_time)
                        self.logger.info(f"Streaming FPS: {fps:.2f}, Total frames: {self.frames_sent}")
                        self.last_stats_time = current_time
                        
                except queue.Empty:
                    await asyncio.sleep(0.01)  # Small delay when no images
                    
            except Exception as e:
                self.logger.error(f"Error in image streamer: {e}")
                await asyncio.sleep(0.1)
                
    def start_server_thread(self):
        """Start the WebSocket server in a separate thread."""
        def run_server():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            
            # Start the WebSocket server
            start_server = websockets.serve(
                self.client_handler,
                self.host,
                self.port,
                ping_interval=20,
                ping_timeout=10,
                max_size=10**7,  # 10MB max message size for large images
                origins=["null"]
            )
            
            self.server = self.loop.run_until_complete(start_server)
            self.logger.info(f"WebSocket server started on ws://{self.host}:{self.port}")
            
            # Start image streamer task
            self.loop.create_task(self.image_streamer())
            
            # Run the event loop
            try:
                self.loop.run_forever()
            except KeyboardInterrupt:
                self.logger.info("WebSocket server stopping...")
            finally:
                self.loop.close()
                
        # Start server in separate thread
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()
        return server_thread
        
    def stop_server(self):
        """Stop the WebSocket server."""
        if self.server and self.loop:
            self.loop.call_soon_threadsafe(self.server.close)
            self.loop.call_soon_threadsafe(self.loop.stop)
            
    def get_connected_clients_count(self) -> int:
        """Get the number of connected clients."""
        return len(self.clients)
        
    def send_system_message(self, message: str, msg_type: str = "system"):
        """Send a system message to all connected clients."""
        if self.loop and self.clients:
            msg = {
                "type": msg_type,
                "message": message,
                "timestamp": time.time()
            }
            asyncio.run_coroutine_threadsafe(
                self.broadcast_to_all(msg), 
                self.loop
            )