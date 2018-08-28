#include "BHandAppHelper.h"

#include "BHand.h"

int handInitWithMenu(BHand *hand, BHCommunication *communication)
{
	bool serial = false;

	if (strcmp(hand->getHardwareDesc()->getModelNumber(), "BH8-262") == 0)
	{
		serial = true;
		printf("\n\n\t\tThe BarrettHand configuration is set to run a 262 hand\n");
	}
	else if (strcmp(hand->getHardwareDesc()->getModelNumber(), "BH8-280") == 0)
	{
		printf("\n\n\t\tThe BarrettHand configuration is set to run a 280 hand\n");
	}
	else
	{
		printf("The BarrettHand configuration is unknown\n");
		return -1;
	}


	// Detemine from user whether connection with hand is serial or CAN
/*	printf("\n\n\r\t\tConnect using:");
	printf("\n\n\r\t\t\t1) Serial");
	printf("\n\n\r\t\t\t2) CAN\n");

	//bool serial = false;
	char ch = ' ';
	while (ch != '1' && ch != '2')
	{
		printf("\n\r\t\tEnter selection (1 or 2) > ");
		ch = (char)_getch();
		printf("%c\n", ch);
		if (ch == '1')
			serial = true;
	}

	// Write the communication method to the parameter passed into this method
	if (communication != NULL)
		*communication = serial ? BH_SERIAL_COMMUNICATION : BH_CAN_COMMUNICATION;*/

	// Initialize the connection with the hand
	if (serial)
	{
		int port = 1;
		int nSuccess = EOF;
		printf("\n\n\r\t\tPlease enter desired serial port> ");
		nSuccess = scanf("%d", &port);
		if (nSuccess != 1)
			return -9;
		printf("\n");
		return hand->Init(port, THREAD_PRIORITY_TIME_CRITICAL, BH_SERIAL_COMMUNICATION);
	}
	else
	{
		printf("\n\n\r\t\tConnecting to hand with CAN\n");
		return hand->Init(0, THREAD_PRIORITY_TIME_CRITICAL, BH_CAN_COMMUNICATION);
	}
}
