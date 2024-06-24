import socket

def receive_values(save_as, port):
    # Create a socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('', port))
    server_socket.listen(1)
    
    print("Listening on port {}...".format(port))

    while True:
        conn, addr = server_socket.accept()
        print("Connection from {}".format(addr))

        data = []
        while True:
            chunk = conn.recv(1024)
            if not chunk:
                break
            data.append(chunk)
        
        data = b''.join(data).decode('utf-8')
        
        # Assuming data format is "overall_tolerance,xy_tolerance,h_tolerance"
        try:
            overall_tolerance, xy_tolerance, h_tolerance = map(float, data.split(','))
        except ValueError:
            print("Received invalid data: {}".format(data))
            conn.close()
            continue
        
        with open(save_as, 'w') as file:
            file.write("{:.6f}, {:.6f}, {:.6f}\n".format(overall_tolerance, xy_tolerance, h_tolerance))
        
        print("Values received and saved: {:.6f}, {:.6f}, {:.6f}".format(overall_tolerance, xy_tolerance, h_tolerance))
        conn.close()

if __name__ == "__main__":
    SAVE_AS = '/home/uvify/catkin_ws/src/survey_mission/path/tolerances.txt'
    PORT = 12346
    
    receive_values(SAVE_AS, PORT)
