/**
 * @file main.cpp
 * @brief Minimal HTTP server for network configuration testing.
 *
 * This module utilizes standalone ASIO to open a synchronous TCP socket,
 * listen for incoming HTTP GET requests, and return a basic plaintext response.
 * It serves as a verification step for the aarch64 cross-compilation and
 * QEMU user-space emulation pipeline.
 *
 * @author Duẫy
 */

#include <iostream>
#include <string>
#include <asio.hpp>

/**
 * @brief Handles a single client connection.
 *
 * Reads the incoming HTTP request headers until a double carriage-return
 * line-feed is encountered, then transmits a standard HTTP 200 OK response.
 *
 * @param socket The active TCP socket connected to the client.
 */
void handle_client(asio::ip::tcp::socket &socket)

{
    try
    {
        asio::streambuf request;
        asio::read_until(socket, request, "\r\n\r\n");

        std::string response =
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/plain; charset=utf-8\r\n"
            "Connection: close\r\n"
            "\r\n"
            "xin chào\n";

        asio::write(socket, asio::buffer(response));
    }
    catch (const std::exception &e)
    {
        std::cerr << "Client connection error: " << e.what() << "\n";
    }
}

/**
 * @brief Entry point for the test server.
 *
 * Initializes the ASIO context, binds to port 8080, and enters an
 * infinite loop to accept and process incoming TCP connections.
 */
int main()
{
    try
    {
        asio::io_context io_context;
        asio::ip::tcp::acceptor acceptor(io_context, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), 8080));

        std::cout << "Test server listening on port 8080...\n";

        while (true)
        {
            asio::ip::tcp::socket socket(io_context);
            acceptor.accept(socket);
            handle_client(socket);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Server initialization failure: " << e.what() << "\n";
        return 1;
    }

    return 0;
}