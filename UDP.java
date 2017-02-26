import java.net.*;

class UDPServer {
	public static void main(String args[]) throws Exception {
		@SuppressWarnings("resource")
		DatagramSocket serverSocket = new DatagramSocket(9876);
		byte[] receiveData = new byte[1024];
		byte[] sendData = new byte[1024];

		while (true) {
			DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);
			serverSocket.receive(receivePacket);
			String sentence = new String(receivePacket.getData());
			System.out.println("RECEIVED: " + sentence);
			InetAddress IPAddress = receivePacket.getAddress();
			int port = receivePacket.getPort();

			String sentenceUpper = sentence.toUpperCase();
		
			int n=1; //Set this to the id of the camera you want....0 is high target camera, 1 is low target camera (IDK why)
			sendData = String.valueOf(n).getBytes();

			DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, IPAddress, port);
			serverSocket.send(sendPacket);
			System.out.println("SENT: " + String.valueOf(n));

		}
	}
}