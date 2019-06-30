#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>

#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <algorithm>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <map>
#include <chrono>
using namespace std::chrono;


#define MAX_INCOMING_QUEUE 10
#define PORT_NUMBER 54321
#define MAX_PACKET_SIZE_BYTES 1048576

#define ANALYZER_TOTAL_PACKETS 10
//#define ANALYZER_TOTAL_PACKETS 100
#define ANALYZER_PACKET_SIZE 1500

#define DEBUG true

class Server {
    int welcome_socket;
    char * read_buffer;
    bool keep_loop_select = true;
    std::string speed_str = "iNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKETiNetworkAnalyzer_SAMPLE_PACKET";

    // clients sockets
    std::map<std::string, int> clients_sockets;
    fd_set clients_fds;
    fd_set read_fds;

    int scnt = 0;
    int rcnt = 0;

public:
    //// C-tor
    Server();
    //// server actions
    void selectPhase();
    void downloadTest(int client_fd);
    void uploadTest(int client_fd);
//    void echoClient(int client_fd);
    void killServer();

private:
    int getMaxFd();
    static void print_error(const std::string& function_name, int error_number);
};


/**
 * Server constructor. Setup welcome socket and waiting until client to connect.
 */
Server::Server() {
    /* setup sockets and structs and listening to welcome socket */
    struct sockaddr_in server_address;

    bzero(&server_address, sizeof(struct sockaddr_in));

    welcome_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (welcome_socket < 0) { print_error("socket() error", errno); }

    int enable = 1;
    //todo SO_REUSEADDR for windows
//    if (setsockopt(welcome_socket, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0){
    //todo SO_REUSEPORT for linux
    if (setsockopt(welcome_socket, SOL_SOCKET, SO_REUSEPORT, &enable, sizeof(int)) < 0){
        print_error("setsockopt", errno);
    }

    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = INADDR_ANY;
    server_address.sin_port = htons(PORT_NUMBER);

    int ret_value = bind(welcome_socket, (struct sockaddr*) &server_address, sizeof(server_address));
    if (ret_value < 0) { print_error("bind", errno); }

    ret_value = listen(welcome_socket, MAX_INCOMING_QUEUE);
    if (ret_value < 0) { print_error("listen", errno); }

    this->read_buffer = new char[MAX_PACKET_SIZE_BYTES + 1];
    bzero(this->read_buffer, MAX_PACKET_SIZE_BYTES + 1);
}


int Server::getMaxFd() {
    int maxfd = this->welcome_socket;
    for (auto client : this->clients_sockets) {
        if (client.second > maxfd) { maxfd = client.second; }
    }
    return maxfd;
}


void Server::selectPhase() {
    if (DEBUG) { printf("DEBUG: %s welcome socket=%d\n", "select phase", welcome_socket); }
    /* initializing arguments for select() */
    FD_ZERO(&clients_fds);
    FD_SET(welcome_socket, &clients_fds);

    int num_ready_incoming_fds = 0;

    while (keep_loop_select) {
        int max_fd = getMaxFd();
        read_fds = clients_fds;
        struct timeval tv_timeout = {30, 0}; // 30 seconds time out
        num_ready_incoming_fds = select((max_fd + 1), &read_fds, nullptr, nullptr, &tv_timeout);

        if (num_ready_incoming_fds == -1) {
            // select error
            print_error("select", errno);

        } else if (num_ready_incoming_fds == 0) {
            if (DEBUG) { printf("**time out\n"); }
            // None of the incoming fds is ready = timeout!
            if (this->clients_sockets.empty()) {
                // no clients kill server
                if (DEBUG) { printf("**no clients!!\n"); }
                //keep_loop_select = false;
                //break;

            } else {
                continue;
            }
        }

        /* at least one incoming fd is ready */

        if (FD_ISSET(welcome_socket, &read_fds)) {
            /* new client arrived welcome */
            // accepting socket - server will enter BLOCKING STATE until client connects.
            int new_client_socket = accept(welcome_socket, nullptr, nullptr);
            if (new_client_socket < 0) {
                print_error("accept", errno);

            } else {
                FD_SET(new_client_socket, &clients_fds);
                clients_sockets.emplace(std::to_string(new_client_socket), new_client_socket);
                //todo
		if (DEBUG) { printf("**new client connected\n"); }
            }
        }

        /* checking other sockets */
        if (!this->clients_sockets.empty()) {
            auto temp_client_sockets = this->clients_sockets;
            for (auto client: temp_client_sockets) {
                if (FD_ISSET(client.second, &read_fds)) {

                    /* Echo back client's messages */
//                    echoClient(client.second);
                    uploadTest(client.second);
                }
            }
        }
    }

}

/**
 * This method will perform the download test by sending the client 1000 packets.
 * After finishing, will close the connection and kill server.
 * @param client_fd client file descriptor
 */
void Server::downloadTest(int client_fd) {
    int ret_value = 0;
    unsigned int bytes_sent;
    strcpy(this->read_buffer, speed_str.c_str());
    if (DEBUG) { printf("DOWNLOAD TEST \n"); }


    while (this->scnt < ANALYZER_TOTAL_PACKETS) {
        bytes_sent = 0;
        if (DEBUG) { printf("#packets sent: %d\n", this->scnt); }

        while (bytes_sent < ANALYZER_PACKET_SIZE) {
            ret_value = send(client_fd, this->read_buffer, ANALYZER_PACKET_SIZE, 0);
            if (ret_value < 0) { print_error("send() failed", errno); }
            bytes_sent = bytes_sent + ret_value;
        }
        this->scnt++;
    }
    if (DEBUG) { printf("#packets sent: %d\n", this->scnt); }
    if (DEBUG) { printf("DOWNLOAD TEST FINISHED\n"); }

    // close client socket
    FD_CLR(client_fd, &this->clients_fds);

    ret_value = shutdown(client_fd, SHUT_RDWR);
    if (ret_value < 0) { print_error("shutdown() failed.", errno); }

    ret_value = close(client_fd);
    if (ret_value < 0) { print_error("close() failed.", errno); }

    this->clients_sockets.erase(std::to_string(client_fd));
    if (DEBUG) { printf("**erasing %d\n", client_fd); }

    if (this->clients_sockets.empty()) {
        if (DEBUG) { printf("**client_sockets empty\n"); }

    }
}

/**
 * This method will consume an client packet.
 * When 1000 packets received, will initiate download test.
 * @param client_fd client file descriptor
 */
void Server::uploadTest(int client_fd) {
    unsigned int bytes_received = 0;
    ssize_t ret_value;

    while (bytes_received < ANALYZER_PACKET_SIZE) {
        ret_value = recv(client_fd, this->read_buffer, (size_t) MAX_PACKET_SIZE_BYTES, 0);
        if (ret_value < 0) { print_error("recv() failed", errno); }

        if (DEBUG) { printf("#%ld bytes received: %s\n", ret_value, this->read_buffer); }
        memset(this->read_buffer, '\0', ret_value);
        bytes_received = bytes_received + ret_value;
    }

    this->rcnt++;
    if (DEBUG) { printf("#packets received %d\n", this->rcnt); }

    if (this->rcnt >= ANALYZER_TOTAL_PACKETS) {
        if (DEBUG) { printf("UPLOAD FINISHED\n"); }

        ret_value = send(client_fd, "UPLOAD_FINISHED\r\n", 15, 0);
        if (ret_value < 0) { print_error("send() failed", errno); }
        memset(this->read_buffer, '\0', ret_value);
        return downloadTest(client_fd);
    }
}
///**
// * This method will echo to specific client fd
// * @param client_fd client socket fd to echo.
// */
//void Server::echoClient(int client_fd) {
//    ssize_t ret_value = recv(client_fd, this->read_buffer, (size_t) MAX_PACKET_SIZE_BYTES, 0);
//    if (ret_value < 0) { print_error("recv() failed", errno); }
//
//	if (DEBUG) { printf("received: %s\n", this->read_buffer); }
//
//    /* return value == 0:
//     * Means we didn't read anything from client - we assume client closed the socket.
//     * Quote from recv() manual:
//     * When a stream socket peer has performed an orderly shutdown,
//     * the return value will be 0 (the traditional "end-of-file" return). */
//    if (ret_value == 0) {
//        // close client socket
//        FD_CLR(client_fd, &this->clients_fds);
//
//        ret_value = shutdown(client_fd, SHUT_RDWR);
//        if (ret_value < 0) { print_error("shutdown() failed.", errno); }
//
//        ret_value = close(client_fd);
//        if (ret_value < 0) { print_error("close() failed.", errno); }
//
//        this->clients_sockets.erase(std::to_string(client_fd));
//        if (DEBUG) { printf("**erasing %d\n", client_fd); }
//        if (this->clients_sockets.empty()) {
//            if (DEBUG) { printf("**client_sockets empty\n"); }
//
//        }
//
//    } else {
//        ret_value = send(client_fd, this->read_buffer, (size_t) ret_value, 0);
//        if (ret_value < 0) { print_error("send() failed", errno); }
//        memset(this->read_buffer, '\0', ret_value);
//    }
//
//}


/**
 * close the open sockets.
 */
void Server::killServer() {
    if (DEBUG) { printf("**killing server\n"); }
    /* deleted buffer */
    delete(this->read_buffer);

    fflush(stdout); //todo delete

    /* close welcome socket */
    FD_CLR(this->welcome_socket, &this->clients_fds);
    shutdown(this->welcome_socket, SHUT_RDWR);
    close(this->welcome_socket);
}


/**
 * This method will print the function that raised the given error number.
 * @param function_name function that caused error
 * @param error_number errno that been raised.
 */
void Server::print_error(const std::string& function_name, int error_number) {
    printf("ERROR: %s %d.\n", function_name.c_str(), error_number);
    exit(EXIT_FAILURE);
}

int main() {

    /* Create server object and listening to clients on welcome socket */
    Server server =  Server();

    if (DEBUG) { printf("**DEBUG: %s\n", "server has been created."); }

    /* Server select loop phase for handling (new/ old/ leaving) clients */
    server.selectPhase();


    /* Close open sockets and close server */
    server.killServer();
    // TODO time out if no clients arrived on time ?
    return EXIT_SUCCESS;
}
