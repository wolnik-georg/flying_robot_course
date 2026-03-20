import socket

s = socket.socket()
s.settimeout(15)
s.connect(("192.168.4.1", 5000))
print("connected, waiting...")
try:
    d = s.recv(4)
    print("GOT DATA:", d.hex())
except socket.timeout:
    print("NOTHING received - GAP8 is not streaming at all")
