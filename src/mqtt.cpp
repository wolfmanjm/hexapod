#include <stdio.h>
#include <string.h>
#include <functional>

#include <mosquitto.h>

static struct mosquitto *mosq = NULL;
static std::function<bool(const char *)> cb;

void my_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message)
{
	if(message->payloadlen){
		//printf("MQTT: %s %s\n", message->topic, message->payload);
		bool exit;

		char msg[message->payloadlen+1];
		memcpy(msg, message->payload, message->payloadlen);
		msg[message->payloadlen]= '\0';
		exit= !cb(msg);

		if(exit) {
			mosquitto_disconnect(mosq);
			mosquitto_loop_stop(mosq, true);
			mosquitto_destroy(mosq);
			mosquitto_lib_cleanup();
		}

	}else{
		//printf("MQTT: %s (null)\n", message->topic);
	}
	fflush(stdout);
}

void my_connect_callback(struct mosquitto *mosq, void *userdata, int result)
{
	int i;
	if(!result){
		/* Subscribe to broker information topics on successful connect. */
		mosquitto_subscribe(mosq, NULL, "quadruped/commands", 2);
	}else{
		fprintf(stderr, "MQTT: Connect failed\n");
	}
}

void my_subscribe_callback(struct mosquitto *mosq, void *userdata, int mid, int qos_count, const int *granted_qos)
{
	int i;

	printf("MQTT: Subscribed (mid: %d): %d", mid, granted_qos[0]);
	for(i=1; i<qos_count; i++){
		printf(", %d", granted_qos[i]);
	}
	printf("\n");
}

void my_log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str)
{
	/* Pring all log messages regardless of level. */
	//printf("MQTT: LOG: %s\n", str);
}

int mqtt_start(const char *host, std::function<bool(const char *)> tcb)
{
	char id[64];
	int i;
	int port = 1883;
	int keepalive = 60;
	bool clean_session = true;
	struct mosquitto *mosq = NULL;

	cb= tcb;
	strcpy(id, "quadruped");

	mosquitto_lib_init();
	mosq = mosquitto_new(id, clean_session, NULL);
	if(!mosq){
		fprintf(stderr, "MQTT: Error: Out of memory.\n");
		return 1;
	}
	mosquitto_log_callback_set(mosq, my_log_callback);

	mosquitto_connect_callback_set(mosq, my_connect_callback);
	mosquitto_message_callback_set(mosq, my_message_callback);
	mosquitto_subscribe_callback_set(mosq, my_subscribe_callback);

	if(mosquitto_connect(mosq, host, port, keepalive)){
		fprintf(stderr, "MQTT: Unable to connect.\n");
		return 1;
	}

	if(mosquitto_loop_start(mosq) != MOSQ_ERR_SUCCESS){
		fprintf(stderr, "MQTT: Unable to start loop.\n");
		return 1;
	}

	return 0;
}

