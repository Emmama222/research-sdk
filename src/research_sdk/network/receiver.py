""" Receiver - Socket
This file includes : 
1. Receiver - Basic UDP receiver w/ Recieving Functions (exc. sending)
2. Broadcast - USP Broadcast receiver 
3. Multicast - UDP Multicast receiver
"""
import socket
import struct
import time

from research_sdk.network.baseUDP import BaseSocket,SocketType
from multiprocessing import Event

#logging
import logging
log = logging.getLogger()
log.setLevel(logging.INFO)

class Receiver(BaseSocket):
    def __init__(self, is_running:Event, type=SocketType.SOCK_UDP, ip = None, port = 0, buffer_size = 1024,timeout=1):
        super().__init__(type=type, ip=ip, port=port, binding=True) #UDP must Bind
        self.is_running:Event = is_running
        self.buffer_size = buffer_size
        self.timeout=timeout
        self.sock.settimeout(timeout)
    
    def __repr__(self) -> str:
        return f"{self.__class__.__name__} - recv @ {self.addr}, available = {self.is_ready}, {self.timeout=}"
    
    
    def listen(self) -> tuple[str, tuple[str, int]] | tuple[None,None]:
        """
        Listens for incoming UDP messages. Returns the message and sender address.
        """
        assert self.is_ready, "Socket is not initialized. try another port ?"
        # default
        result = None,None
        while self.is_running.is_set():
            try:
                # listen
                message, addr = self.sock.recvfrom(self.buffer_size)
                # print(message)
                # decode
                decoded_message = self._decode(message)
                # update variable
                result = decoded_message, addr
                return result 
            except socket.timeout:
                # print("timeout")
                continue
            except KeyboardInterrupt:
                continue
        
        print(f"{self.__class__.__name__} has stopped listening")
        return result
    
    def listen_for(self,duration:int) -> tuple[str, tuple[str, int]] | tuple[None,None] :
        result = None,None
        end_time = time.time() + duration
        while time.time() < end_time:
            try:
                # listen
                message, addr = self.sock.recvfrom(self.buffer_size)
                # decode
                last_message = self._decode(message)
                # update variable
                result = last_message, addr
                active = False
            except socket.timeout:
                # print("timeout")
                continue
            except KeyboardInterrupt:
                break
        return result
                

    def _decode(self,data:bytes):
        try:
            decoded_data = data.decode()
        except UnicodeDecodeError:
            raise UnicodeDecodeError(f"cannot decode {data} with default 'utf-8'")
        return decoded_data

class Broadcast(Receiver):
    """reciever for broadcast"""
    def __init__(self, port: int = 0, buffer_size: int = 1024):
        ip = "0.0.0.0" #this is the broadcast address
        type = SocketType.SOCK_BROADCAST_UDP
        super().__init__(ip=ip,port=port,type=type,buffer_size=buffer_size)
        
    

# Multicast Receiver
class SSL_Multicast(Receiver):
    def __init__(self,is_running:Event,port: int, group:str, decoder:object, buffer_size: int = 6000,timeout:float=0.5) -> None:
        """
        Initialising Multicast socket

        Args:
            is_running (Event): multiprocessing event that controls operation if it is running
            port (int): port of multicast connection. 
            group (str): Mulitcast group. Defaults to "224.5.0.0" -> nothing.
            decoder (object): SSL Protobuf file and descriptor to decode data
            buffer_size (int, optional): buffer_size for listening. Defaults to 6000. 
            timeout (float, optional): 

        Raises:
            Exception: Needed decoder
        """
        super().__init__(is_running=is_running,ip="",port=port,type=SocketType.SOCK_MULTICAST_UDP,buffer_size=buffer_size,timeout=timeout)

        self.group : str = group
        self.decoder:object = decoder
        if self.decoder is None:
            raise AttributeError ("MISSING DECODER")
        self._add_group()
            
    
    @staticmethod
    def _get_vision_ip() -> str:
        # """Get vision network interface IP from ipconfig.yaml, fallback 0.0.0.0 (all interfaces)."""
        # try:
            from research_sdk.utils.yaml_config import Config
            cfg = Config()
            return cfg.vision_ip
        # except Exception:
        #     return "0.0.0.0"
        #     return "0.0.0.0"

    def _add_group(self):
        """Joins multicast group, trying the configured interface then falling back to INADDR_ANY."""
        self.is_ready = False
        ip = self._get_vision_ip()
        print(f"[Multicast] joining {self.group} on {ip}")
        for iface in ([ip, "0.0.0.0"] if ip != "0.0.0.0" else ["0.0.0.0"]):
            try:
                mreq = struct.pack("=4s4s", socket.inet_aton(self.group), socket.inet_aton(iface))
                self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
                print(f"[Multicast] joined {self.group} on {iface}")
                self.is_ready = True
                return
            except OSError as e:
                print(f"[Multicast] failed to join {self.group} on {iface}: {e}")
        raise OSError(f"[Multicast] could not join {self.group} on any interface")
        
    def listen(self) -> object | bytearray:
        # set to only return the decoded data
        decoded_info,_ = super().listen()
        return decoded_info
    
    def _decode(self,data:bytes) -> bool:
        """
        Reading and decoding the data (overrides super)

        Args:
            data (bytes): data received from listen ()

        Returns:
            str: decoded data
        """
        # Decode with Protobuf 
        try:
            decoded_data:str= self.decoder.FromString(data) 
            return decoded_data
        except Exception as e:
            print(f"message cannot be decoded by {self.decoder=} : {data}")
    
class SSL_Broadcast(Receiver):
    
    def __init__(self,is_running:Event,port: int, group:str, decoder:object, buffer_size: int = 6000,timeout:float=0.5) -> None:
        """
        Initialising Multicast socket

        Args:
            is_running (Event): multiprocessing event that controls operation if it is running
            port (int): port of multicast connection. 
            group (str): Mulitcast group. Defaults to "224.5.0.0" -> nothing.
            decoder (object): SSL Protobuf file and descriptor to decode data
            buffer_size (int, optional): buffer_size for listening. Defaults to 6000. 
            timeout (float, optional): 

        Raises:
            Exception: Needed decoder
        """
        super().__init__(is_running=is_running,ip="",port=port,type=SocketType.SOCK_BROADCAST_UDP,buffer_size=buffer_size,timeout=timeout)

        self.group : str = group
        self.decoder:object = decoder
        if self.decoder is None:
            raise AttributeError ("MISSING DECODER")

    def listen(self) -> object | bytearray:
        # set to only return the decoded data
        decoded_info,_ = super().listen()
        return decoded_info
    
    def _decode(self,data:bytes) -> bool:
        """
        Reading and decoding the data (overrides super)

        Args:
            data (bytes): data received from listen ()

        Returns:
            str: decoded data
        """
        # Decode with Protobuf 
        try:
            decoded_data:str= self.decoder.FromString(data) 
            return decoded_data
        except Exception as e:
            print(f"message cannot be decoded by {self.decoder=} : {data}")

if __name__ == "__main__":
    is_running = Event()
    recv = Receiver(is_running,port = 50514)
    b_recv = Broadcast()
    m_recv = SSL_Multicast(is_running,10006, "224.24.5.3", '')
    while True:
        print("receiver is listening")
        msg,_ = recv.listen()
        print(msg)
        # print("broadcast is listening")
        # b_recv.listen()
        # print("multicast is listening")
        # m_recv.listen()