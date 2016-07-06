//
//  Weather update client in C++
//  Connects SUB socket to tcp://localhost:5556
//  Collects weather updates and finds avg temp in zipcode
//
//  Olivier Chamoux <olivier.chamoux@fr.thalesgroup.com>
//
/*
 #include <zmq.hpp>
 #include <iostream>
 #include <sstream>

int main (int argc, char *argv[])
 {
    zmq::context_t context (1);

    //  Socket to talk to server
    std::cout << "Collecting updates from weather server…\n" << std::endl;
    try
    {
        zmq::socket_t subscriber (context, ZMQ_SUB);
        subscriber.connect("tcp://127.0.0.1:5556");

        //  Subscribe to zipcode, default is NYC, 10001
        const char *filter = (argc > 1)? argv [1]: "10001 ";
        subscriber.setsockopt(ZMQ_SUBSCRIBE, filter, strlen (filter));

        //  Process 100 updates
        int update_nbr;
        long total_temp = 0;
        for (update_nbr = 0; update_nbr < 100; update_nbr++) {

            zmq::message_t update;
            int zipcode, temperature, relhumidity;

            subscriber.recv(&update);

            std::istringstream iss(static_cast<char*>(update.data()));
            iss >> zipcode >> temperature >> relhumidity ;

            total_temp += temperature;

            printf("%s\n", (char *)update.data());
        }
        std::cout     << "Average temperature for zipcode '"<< filter
                    <<"' was "<<(int) (total_temp / update_nbr) <<"F"
                    << std::endl;
    }
    catch (zmq::error_t& err)
    {
        fprintf(stderr, "Error: %s\n", err.what());
    }

    return 0;
 }
*/

#include <zhelpers.h>

int main (int argc, char *argv [])
 {
    //  Socket to talk to server
    printf ("Collecting updates from weather server…\n");
    void *context = zmq_ctx_new ();
    void *subscriber = zmq_socket (context, ZMQ_SUB);
    int rc = zmq_connect (subscriber, "tcp://127.0.0.1:5556");
    assert (rc == 0);

    //  Subscribe to zipcode, default is NYC, 10001
    char *filter = (argc > 1)? argv [1]: "10001 ";
    rc = zmq_setsockopt (subscriber, ZMQ_SUBSCRIBE, "", 0);
    assert (rc == 0);

    //  Process 100 updates
    int update_nbr;
    long total_temp = 0;
    for (update_nbr = 0; update_nbr < 100; update_nbr++) {
        printf("Receiving..");
        char *string = s_recv (subscriber);
        printf(" Received %s\n", string);

        int zipcode, temperature, relhumidity;
        sscanf (string, "%d %d %d",
            &zipcode, &temperature, &relhumidity);
        total_temp += temperature;
        free (string);
    }
    printf ("Average temperature for zipcode '%s' was %dF\n",
        filter, (int) (total_temp / update_nbr));

    zmq_close (subscriber);
    zmq_ctx_destroy (context);
    return 0;
 }
