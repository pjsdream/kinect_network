//
//  Weather update server in C++
//  Binds PUB socket to tcp://*:5556
//  Publishes random weather updates
//
//  Olivier Chamoux <olivier.chamoux@fr.thalesgroup.com>
//
/*
#include <zmq.hpp>
 #include <stdio.h>
 #include <stdlib.h>
 #include <time.h>

#define within(num) (int) ((float) num * rand () / (RAND_MAX + 1.0))

int main () {

    //  Prepare our context and publisher
    try
    {
        zmq::context_t context (1);
        zmq::socket_t publisher (context, ZMQ_PUB);
        publisher.bind("tcp://127.0.0.1:5556");

        //  Initialize random number generator
        srand ((unsigned) time (NULL));
        while (1) {

            int zipcode, temperature, relhumidity;

            //  Get values that will fool the boss
            zipcode     = within (100000);
            temperature = within (215) - 80;
            relhumidity = within (50) + 10;

            //  Send message to all subscribers
            zmq::message_t message(20);
            snprintf ((char *) message.data(), 20 ,
                "%05d %d %d", zipcode, temperature, relhumidity);
            publisher.send(message);
            printf("%s\n", message.data());

        }
    }
    catch (zmq::error_t& err)
    {
        fprintf(stderr, "Error: %s\n", err.what());
    }

    return 0;
 }
*/

#include <zhelpers.h>

int main (void)
 {
    //  Prepare our context and publisher
    void *context = zmq_ctx_new ();
    void *publisher = zmq_socket (context, ZMQ_PUB);
    int rc = zmq_bind (publisher, "tcp://*:5556");
    assert (rc == 0);

    //  Initialize random number generator
    srandom ((unsigned) time (NULL));
    while (1) {
        //  Get values that will fool the boss
        int zipcode, temperature, relhumidity;
        zipcode     = randof (100000);
        temperature = randof (215) - 80;
        relhumidity = randof (50) + 10;

        //  Send message to all subscribers
        char update [20];
        sprintf (update, "%05d %d %d", zipcode, temperature, relhumidity);
        s_send (publisher, update);

        printf("Sent %s\n", update);
    }
    zmq_close (publisher);
    zmq_ctx_destroy (context);
    return 0;
 }
