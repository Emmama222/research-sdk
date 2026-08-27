""" SENDER
Sender includes :
    1. Sender            - UDP socket for server.
    2. Broadcaster       - UDP broadcast socket for server.
    3. Multicaster       - UDP Multicast socket (Not Implemented)
Raises:
    NotImplementedError: if user try to use an empty function
"""
#logging
import logging
log = logging.getLogger()
log.setLevel(logging.DEBUG)


from research_sdk.network.baseUDP import BaseSocket, SocketType

class Sender(BaseSocket):
    def __init__(self, device_ip, type=SocketType.SOCK_UDP, binding=False) -> None:
        # device_ip = None # use 127.0.0.1 for local testing 
        device_port = self._generate_port()
        super().__init__(ip=device_ip,port=device_port,type=type,binding=binding)
    
    def send(self, msg, ip:str, port:int):
        # update address as provided, otherwise uses default
        try:
            addr = tuple((str(ip),int(port))) 
            if not isinstance(msg,bytes):
                msg = msg.encode()
        except Exception as e:
            raise Exception(type(e), " check address and msg ", e)
        
        self.sock.sendto(msg,addr)
    
    
class LockedSender(BaseSocket):
    def __init__(self, ip: str='127.0.0.1', port: int=0, type=SocketType.SOCK_UDP, binding=False) -> None:
        device_ip = None 
        device_port = self._generate_port()
        self.destination_ip = ip 
        self.destination_port = port
        super().__init__(ip=device_ip,port=device_port,type=type,binding=binding)
    
    @property
    def destination_ip(self):
        return self._destination_ip
    
    @destination_ip.setter
    def destination_ip(self,value):
        if not isinstance(value,str):
            raise TypeError("Need IP Address (v4) as string type")
        self._destination_ip = value
        
    @property
    def destination(self):
        return (self.destination_ip,self.destination_port)
    
    
    def send(self, msg, ip:str=None, port:int=None):
        # update address as provided, otherwise uses default
        addr = tuple(ip,port) if isinstance(ip,str) and isinstance(port,int) else self.destination
        if not isinstance(msg,bytes):
            try:
                msg = msg.encode()
            except Exception as e:
                raise(e, "Error with encoding")
        # print(self.destination)
        self.sock.sendto(msg,addr)
        
        # print(f"sending {msg} to {addr}")

            
    
    def update_destination(self, destination:str|tuple[str,int]) -> None:
        """Update Sending Socket's Destination
        This is a static method, so when the send is trigger, will always send to this specific destination (ip,port)

        Args:
            destination (str | tuple[str,int]): Destination addr, can be in format string / tuple
        """
        if isinstance(destination,str):
            destination = self.string_to_tuple(destination)
        self.destination_ip = destination[0]
        self.port = destination[1]
    
    

class Broadcaster(LockedSender):
    def __init__(self, broadcasting_port: int = 12342) -> None:
        """
        UDP Broadcast sending channel

        Args:
            port (int, optional): Broadcast channel. Defaults to 12342. *This has to be calibrated with recepients
        
        Params:
            ip (str) : Default broadcast. See python udp_broadcast for more information.
        """
        ip = '<broadcast>'
        type =SocketType.SOCK_BROADCAST_UDP
        super().__init__(ip=ip,port=broadcasting_port,type=type,binding=True)
        
        
class Multicaster(Sender):
    ...

if __name__ == "__main__":
    import time
    sender = Sender(ip='', port=50514)
    while True:
        # time.sleep(2)
        for i in range (500):
            sender.send(str(i))