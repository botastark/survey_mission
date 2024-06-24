import socket
def receive_file(save_as, port):

	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
	s.bind(('', port)) 
	s.listen(1) 
	print("Listening on port {}...".format(port)) 
	conn, addr = s.accept() 
	print("Connection from {}".format(addr)) # Receive the data 
	file_data = conn.recv(1024).decode('utf-8') # Save the received data to a file 
	with open(save_as, 'w') as file: 
		file.write(file_data) # Close the connection 
	conn.close() 
if __name__ == "__main__": 
	SAVE_AS = '/home/uvify/received_file.txt' 
	PORT = 12345 # This should match the port used by the sender 
	receive_file(SAVE_AS, PORT)
