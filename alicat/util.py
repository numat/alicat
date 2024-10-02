"""Base functionality for async communication.

Distributed under the GNU General Public License v2
Copyright (C) 2024 Alex Ruddick
Copyright (C) 2023 NuMat Technologies
"""
from __future__ import annotations

import asyncio
import logging
from abc import ABC, abstractmethod
from typing import Any

import serial

logger = logging.getLogger('alicat')


class Client(ABC):
    """Serial or TCP client."""

    def __init__(self, timeout: float):
        """Initialize common attributes."""
        self.address = ''
        self.open = False
        self.timeout = timeout
        self.timeouts = 0
        self.max_timeouts = 10
        self.connection: dict[str, Any] = {}
        self.reconnecting = False
        self.eol = b'\r'
        self.lock = asyncio.Lock()

    @abstractmethod
    async def _write(self, message: str) -> None:
        """Write a message to the device."""
        ...

    @abstractmethod
    async def _read(self, length: int) -> str:
        """Read a fixed number of bytes from the device."""

    @abstractmethod
    async def _readline(self) -> str:
        """Read until a LF terminator."""

    async def _write_and_read(self, command: str) -> str | None:
        """Write a command and read a response.

        As industrial devices are commonly unplugged, this has been expanded to
        handle recovering from disconnects.
        """
        await self._handle_connection()
        async with self.lock:
            if self.open:
                try:
                    response = await self._handle_communication(command)
                    return response
                except asyncio.exceptions.IncompleteReadError:
                    logger.error('IncompleteReadError.  Are there multiple connections?')
                    return None
            else:
                return None

    async def _clear(self) -> None:
        """Clear the reader stream when it has been corrupted from multiple connections."""
        logger.warning("Multiple connections detected; clearing reader stream.")
        try:
            junk = await asyncio.wait_for(self._read(100), timeout=0.5)
            logger.warning(junk)
        except TimeoutError:
            pass

    async def _handle_communication(self, command: str) -> str | None:
        """Manage communication, including timeouts and logging."""
        try:
            await self._write(command)
            result = await self._readline()
            self.timeouts = 0
            return result
        except (asyncio.TimeoutError, TypeError, OSError):
            self.timeouts += 1
            if self.timeouts == self.max_timeouts:
                logger.error(f'Reading from {self.address} timed out '
                             f'{self.timeouts} times.')
                await self.close()
            return None

    @abstractmethod
    async def _handle_connection(self) -> None:
        pass

    @abstractmethod
    async def close(self) -> None:
        """Close the connection."""
        pass


class TcpClient(Client):
    """A generic reconnecting asyncio TCP client.

    This base functionality can be used by any industrial control device
    communicating over TCP.
    """

    def __init__(self, address: str, timeout: float=1.0):
        """Communicator using a TCP/IP<=>serial gateway."""
        super().__init__(timeout)
        try:
            self.address, self.port = address.split(':')
        except ValueError as e:
            raise ValueError('address must be hostname:port') from e

    async def __aenter__(self) -> Client:
        """Provide async entrance to context manager.

        Contrasting synchronous access, this will connect on initialization.
        """
        await self._handle_connection()
        return self

    async def __aexit__(self, *args: Any) -> None:
        """Provide async exit to context manager."""
        await self.close()

    async def _connect(self) -> None:
        """Asynchronously open a TCP connection with the server."""
        await self.close()
        reader, writer = await asyncio.open_connection(self.address, self.port)
        self.connection = {'reader': reader, 'writer': writer}
        self.open = True

    async def _read(self, length: int) -> str:
        """Read a fixed number of bytes from the device."""
        await self._handle_connection()
        response = await self.connection['reader'].read(length)
        return response.decode().strip()

    async def _readline(self) -> str:
        """Read until a line terminator."""
        await self._handle_connection()
        response = await self.connection['reader'].readuntil(self.eol)
        return response.decode().strip().replace('\x00', '')

    async def _write(self, command: str) -> None:
        """Write a command and do not expect a response.

        As industrial devices are commonly unplugged, this has been expanded to
        handle recovering from disconnects.
        """
        await self._handle_connection()
        self.connection['writer'].write(command.encode() + self.eol)

    async def _handle_connection(self) -> None:
        """Automatically maintain TCP connection."""
        if self.open:
            return
        try:
            await asyncio.wait_for(self._connect(), timeout=0.75)
            self.reconnecting = False
        except (asyncio.TimeoutError, OSError):
            if not self.reconnecting:
                logger.error(f'Connecting to {self.address} timed out.')
            self.reconnecting = True

    async def _handle_communication(self, command: str) -> str | None:
        """Manage communication, including timeouts and logging."""
        try:
            await self._write(command)
            future = self._readline()
            result = await asyncio.wait_for(future, timeout=0.75)
            self.timeouts = 0
            return result
        except (asyncio.TimeoutError, TypeError, OSError):
            self.timeouts += 1
            if self.timeouts == self.max_timeouts:
                logger.error(f'Reading from {self.address} timed out '
                             f'{self.timeouts} times.')
                await self.close()
            return None

    async def close(self) -> None:
        """Close the TCP connection."""
        if self.open:
            self.connection['writer'].close()
            await self.connection['writer'].wait_closed()
        self.open = False


class SerialClient(Client):
    """Client using a directly-connected RS232 serial device."""

    def __init__(self, address: str, baudrate: int=19200, timeout: float=.15,
                 bytesize: int = serial.EIGHTBITS,
                 stopbits: int = serial.STOPBITS_ONE,
                 parity: str = serial.PARITY_NONE):
        """Initialize serial port."""
        super().__init__(timeout)
        self.address = address
        assert isinstance(self.address, str)
        self.serial_details = {'baudrate': baudrate,
                               'bytesize': bytesize,
                               'stopbits': stopbits,
                               'parity': parity,
                               'timeout': timeout}
        self.ser = serial.Serial(self.address, **self.serial_details)  # type: ignore [arg-type]

    async def _read(self, length: int) -> str:
        """Read a fixed number of bytes from the device."""
        return self.ser.read(length).decode()

    async def _readline(self) -> str:
        """Read until a LF terminator."""
        return self.ser.readline().strip().decode().replace('\x00', '')

    async def _write(self, message: str) -> None:
        """Write a message to the device."""
        self.ser.write(message.encode() + self.eol)

    async def close(self) -> None:
        """Release resources."""
        self.ser.close()

    async def _handle_connection(self) -> None:
        self.open = True

def _is_float(msg: Any) -> bool:
    try:
        float(msg)
        return True
    except ValueError:
        return False
