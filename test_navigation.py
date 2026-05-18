#!/usr/bin/env python3
"""
Simple test script for NAO Voice Navigation
Sends destination commands via TCP to the NAO controller
"""

import socket

HOST = "127.0.0.1"
PORT = 5005

def main():
    print("NAO Navigation Test")
    print("=" * 40)
    print("Commands: kitchen, bedroom, living room, bathroom, home, stop")
    print("Type 'quit' to exit")
    print("=" * 40)
    print()

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((HOST, PORT))
        print(f"Connected to NAO controller at {HOST}:{PORT}\n")
    except ConnectionRefusedError:
        print("ERROR: Could not connect to NAO controller.")
        print("Make sure the Webots simulation is running!")
        return

    try:
        while True:
            cmd = input("Enter destination: ").strip().lower()
            if cmd == "quit":
                break
            if cmd:
                sock.sendall((cmd + "\n").encode())
                print(f"Sent: {cmd}\n")
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        sock.close()

if __name__ == "__main__":
    main()
