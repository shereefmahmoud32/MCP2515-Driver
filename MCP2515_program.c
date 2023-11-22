/*********************************************************************************/
/*********************************************************************************/
/**********************			Author: Shereef Mahmoud		**********************/
/**********************			File: MCP2515_program.c		**********************/
/**********************			Version: 1.00				**********************/
/*********************************************************************************/
/*********************************************************************************/

/**
 * @file MCP2515_program.c
 * @author Shereef Mahmoud
 * @brief This file implements the MCP2515 module functions.
 * @version 1.00
 */

#include <stdint.h>
#include "../Inc/BIT_MATH.h"
#include "../Inc/DEFINES.h"

#include "../Inc/GPIO_interface.h"
#include "../Inc/MSPI_interface.h"

#include "../Inc/MCP2515_interface.h"
#include "../Inc/MCP2515_config.h"
#include "../Inc/MCP2515_private.h"

uint8_t MCP2515_u8ReceiveData;


static GPIO_PinConfig_t Main_stSS = {
		.AltFunc = GPIO_AF5,
		.OutputSpeed = GPIO_LOW,
		.OutputType = GPIO_PUSH_PULL,
		.PinMode = GPIO_OUTPUT,
		.PinNum = GPIO_PIN4,
		.Port = GPIO_PORTA,
		.PullType = GPIO_NO_PULL
};


 /**
 * @brief Set the state of the MCP2515 chip select pin.
 *
 * @param[in] Copy_u8State The desired state of the chip select pin.
 *                         - SLAVE_CHOOSE: Select the MCP2515 (assert the chip select pin).
 *                         - SLAVE_RELEASE: Release the MCP2515 (deassert the chip select pin).
 *
 * @note This function controls the chip select pin of the MCP2515, allowing
 *       for selection or release of the SPI slave device.
 */

static void MCP2515_voidChipSelect(uint8_t Copy_u8State)
{
	if(Copy_u8State == SLAVE_CHOOSE)
	{
		GPIO_enSetPinValue(&Main_stSS, GPIO_PIN_LOW);
	}
	else if(Copy_u8State == SLAVE_RELEASE)
	{
		GPIO_enSetPinValue(&Main_stSS, GPIO_PIN_HIGH);
	}
}



 /**
 * @brief Read a register value from the MCP2515.
 *
 * @param[in] Copy_u8Address The address of the register to be read.
 * @return The value read from the specified register.
 *
 * @note This function reads the content of a register in the MCP2515 by following
 *       the SPI communication protocol. It selects the MCP2515, sends the read
 *       command along with the register address, and receives the data from the
 *       specified register. Finally, it releases the MCP2515.
 */

uint8_t MCP2515_u8ReadRegister(uint8_t Copy_u8Address)
{
	uint8_t Local_u8ReadValue;

	MCP2515_voidChipSelect(SLAVE_CHOOSE);

	MSPI_voidSendRecieveDataSync(INST_READ, &MCP2515_u8ReceiveData);
	MSPI_voidSendRecieveDataSync(Copy_u8Address, &MCP2515_u8ReceiveData);
	MSPI_voidSendRecieveDataSync(0, &Local_u8ReadValue);

	MCP2515_voidChipSelect(SLAVE_RELEASE);

	return Local_u8ReadValue;
}


/**
 * @brief Write a value to a register in the MCP2515.
 *
 * @param[in] Copy_u8Address The address of the register to be written.
 * @param[in] Copy_u8Value   The value to be written to the specified register.
 *
 * @note This function writes a value to a register in the MCP2515 by following
 *       the SPI communication protocol. It selects the MCP2515, sends the write
 *       command along with the register address and the data to be written,
 *       and then releases the MCP2515.
 */

void MCP2515_voidWriteRegister(uint8_t Copy_u8Address, uint8_t Copy_u8Value)
{
	MCP2515_voidChipSelect(SLAVE_CHOOSE);

	MSPI_voidSendRecieveDataSync(INST_WRITE, &MCP2515_u8ReceiveData);
	MSPI_voidSendRecieveDataSync(Copy_u8Address, &MCP2515_u8ReceiveData);
	MSPI_voidSendRecieveDataSync(Copy_u8Value, &MCP2515_u8ReceiveData);

	MCP2515_voidChipSelect(SLAVE_RELEASE);
}

/**
 * @brief Set the bit timing configuration for the MCP2515.
 *
 * @param[in] Copy_u8CNF1Value The value to be written to the CNF1 register.
 * @param[in] Copy_u8CNF2Value The value to be written to the CNF2 register.
 * @param[in] Copy_u8CNF3Value The value to be written to the CNF3 register.
 *
 * @note This function configures the bit timing parameters of the MCP2515
 *       by setting the values in the CNF1, CNF2, and CNF3 registers.
 *       It utilizes the MCP2515_voidWriteRegister function for each register.
 */

void MCP2515_voidSetBitTiming(uint8_t Copy_u8CNF1Value, uint8_t Copy_u8CNF2Value, uint8_t Copy_u8CNF3Value)
{
	MCP2515_voidWriteRegister(CNF1, Copy_u8CNF1Value);
	MCP2515_voidWriteRegister(CNF2, Copy_u8CNF2Value);
	MCP2515_voidWriteRegister(CNF3, Copy_u8CNF3Value);
}


/**
 * @brief Reset the MCP2515.
 *
 * @note This function initiates a reset of the MCP2515 by selecting
 *       the device, sending the reset command, and then releasing it.
 *       It uses the MCP2515_voidChipSelect function to control the
 *       chip select pin during the reset process.
 */

void MCP2515_voidReset(void)
{
	MCP2515_voidChipSelect(SLAVE_CHOOSE);

	MSPI_voidSendRecieveDataSync(INST_RESET, &MCP2515_u8ReceiveData);

	MCP2515_voidChipSelect(SLAVE_RELEASE);
}

/**
 * @brief Modify specific bits in a register of the MCP2515.
 *
 * @param[in] Copy_u8RegAddress The address of the register to be modified.
 * @param[in] Copy_u8Mask       The bit mask to apply for modification.
 * @param[in] Copy_u8Value      The new values to set for the specified bits.
 *
 * @note This function modifies specific bits in a register of the MCP2515
 *       using the bit modify command. It allows for selective bit changes
 *       by specifying a bit mask and the desired new values. The chip select
 *       pin is controlled using MCP2515_voidChipSelect during the operation.
 */

void MCP2515_voidChangeBits(uint8_t Copy_u8RegAddress, uint8_t Copy_u8Mask, uint8_t Copy_u8Value)
{
	MCP2515_voidChipSelect(SLAVE_CHOOSE);

	MSPI_voidSendRecieveDataSync(INST_BIT_MODIFY, &MCP2515_u8ReceiveData);
	MSPI_voidSendRecieveDataSync(Copy_u8RegAddress, &MCP2515_u8ReceiveData);
	MSPI_voidSendRecieveDataSync(Copy_u8Mask, &MCP2515_u8ReceiveData);
	MSPI_voidSendRecieveDataSync(Copy_u8Value, &MCP2515_u8ReceiveData);

	MCP2515_voidChipSelect(SLAVE_RELEASE);
}


/**
 * @brief Set the operating mode of the MCP2515.
 *
 * @param[in] Copy_u8Mode The desired operating mode to be set.
 *
 * @note This function sets the operating mode of the MCP2515 by modifying
 *       specific bits in the CANCTRL register. It utilizes MCP2515_voidChangeBits
 *       to perform the necessary bit modification. Additionally, it includes
 *       a polling mechanism to wait until the MCP2515 transitions to the
 *       specified operating mode before returning.
 */

void MCP2515_voidSetMode(uint8_t Copy_u8Mode)
{
	MCP2515_voidChangeBits(CANCTRL, 0xE0, Copy_u8Mode << 5);
	while((MCP2515_u8ReadRegister(CANSTAT) >> 5) != Copy_u8Mode);
}


/**
 * @brief Initialize the MCP2515 module.
 *
 * @note This function initializes the MCP2515 module by configuring
 *       the SPI master, performing a reset, entering configuration mode,
 *       setting the bit timing parameters for a 250 KHz bit rate using
 *       an 8 MHz oscillator, and transitioning to the normal operating mode.
 */

void MCP2515_voidInit(void)
{
	MSPI_voidInitMaster();
	MCP2515_voidReset();

	/*Enter configuration mode and enable CLKOUT with no prescaler*/
	MCP2515_voidWriteRegister(CANCTRL, 0x84);

	while((MCP2515_u8ReadRegister(CANSTAT)>>5) != MCP2515_MODE_CONFG);

	/*To run at 250 KHz bit rate using 8 MHz oscillator*/
	MCP2515_voidSetBitTiming((2 << 6), ((1 << 7) | (6 << 3) | (1)), 5);

	MCP2515_voidSetMode(MCP2515_MODE_NORMAL);
}

/**
 * @brief Send a CAN message through the MCP2515 module.
 *
 * @param[in] Copy_stMessage The CAN message to be sent.
 *
 * @note This function prepares and transmits a CAN message through the MCP2515 module.
 *       It configures the transmit buffer (TXB0) with the provided message details,
 *       initiates a request-to-send (RTS) command, and controls the chip select pin
 *       during the operation using MCP2515_voidChipSelect.
 *
 * @param[in] Copy_stMessage The CAN message to be sent.
 */

void MCP2515_voidSendCANmsg(MCP2515_CanMessage_t Copy_stMessage)
{
	MCP2515_voidChipSelect(SLAVE_CHOOSE);

	uint8_t Local_u8Counter;

	/*Send header and address*/
	MSPI_voidSendRecieveDataSync(INST_WRITE, &MCP2515_u8ReceiveData);
	MSPI_voidSendRecieveDataSync(TXB0CTRL, &MCP2515_u8ReceiveData);

	/*Setup message priority*/
	MSPI_voidSendRecieveDataSync(3, &MCP2515_u8ReceiveData);

	/* Setup standard or extended identifier */
	MSPI_voidSendRecieveDataSync((uint8_t)(Copy_stMessage.id >> 3), &MCP2515_u8ReceiveData);
	MSPI_voidSendRecieveDataSync((uint8_t)(Copy_stMessage.id << 5), &MCP2515_u8ReceiveData);
	MSPI_voidSendRecieveDataSync(0, &MCP2515_u8ReceiveData);
	MSPI_voidSendRecieveDataSync(0, &MCP2515_u8ReceiveData);

	/* Setup message length and RTR bit */
	MSPI_voidSendRecieveDataSync((Copy_stMessage.length) | (Copy_stMessage.properties << 6), &MCP2515_u8ReceiveData);

	/* Store the message into the buffer */
	for(Local_u8Counter = 0; Local_u8Counter < Copy_stMessage.length; Local_u8Counter++)
	{
		MSPI_voidSendRecieveDataSync(Copy_stMessage.data[Local_u8Counter], &MCP2515_u8ReceiveData);
	}

	/*Release the bus*/
	MCP2515_voidChipSelect(SLAVE_RELEASE);

	/*Send request to send*/
	MCP2515_voidChipSelect(SLAVE_CHOOSE);
	MSPI_voidSendRecieveDataSync(INST_RTS_TXB0, &MCP2515_u8ReceiveData);
	MCP2515_voidChipSelect(SLAVE_RELEASE);
}

/**
 * @brief Set the Rollover mode for the MCP2515.
 *
 * @param[in] Copy_u8Value The desired Rollover mode to be set.
 *
 * @note This function sets the Rollover mode for the MCP2515 by modifying
 *       specific bits in the RXB0CTRL register. It uses MCP2515_voidChangeBits
 *       to perform the necessary bit modification.
 */

void MCP2515_voidSetRollover(uint8_t Copy_u8Value)
{
	MCP2515_voidChangeBits(RXB0CTRL, 1 << BUKT, Copy_u8Value << BUKT);
}

/**
 * @brief Set a mask in the MCP2515 module.
 *
 * @param[in] Copy_u8MaskAddress The address of the mask register to be set.
 * @param[in] Copy_u32MaskValue  The value of the mask to be set.
 * @param[in] Copy_u8Extended    Flag indicating if the mask is extended (29-bit) or standard (11-bit).
 *
 * @note This function sets a mask in the MCP2515 module by configuring the specified mask register.
 *       The mask value is determined by the provided parameters, and the chip select pin is controlled
 *       during the operation using MCP2515_voidChipSelect.
 */

void MCP2515_voidSetMask(uint8_t Copy_u8MaskAddress, uint32_t Copy_u32MaskValue, uint8_t Copy_u8Extended)
{
	MCP2515_voidChipSelect(SLAVE_CHOOSE);

	MSPI_voidSendRecieveDataSync(INST_WRITE, &MCP2515_u8ReceiveData);
	MSPI_voidSendRecieveDataSync(Copy_u8MaskAddress, &MCP2515_u8ReceiveData);

	if(Copy_u8Extended)
	{
		/*Extended Address*/
		MSPI_voidSendRecieveDataSync((uint8_t) (Copy_u32MaskValue >> 3), &MCP2515_u8ReceiveData);
		MSPI_voidSendRecieveDataSync((uint8_t) (Copy_u32MaskValue << 5) | (1<<3) | (uint32_t) (Copy_u32MaskValue >> 27), &MCP2515_u8ReceiveData);
		MSPI_voidSendRecieveDataSync((uint8_t) (Copy_u32MaskValue >> 19), &MCP2515_u8ReceiveData);
		MSPI_voidSendRecieveDataSync((uint8_t) (Copy_u32MaskValue >> 11), &MCP2515_u8ReceiveData);
	}
	else
	{
		/*Standard Address*/
		MSPI_voidSendRecieveDataSync((uint8_t) (Copy_u32MaskValue >> 3), &MCP2515_u8ReceiveData);
		MSPI_voidSendRecieveDataSync((uint8_t) (Copy_u32MaskValue << 5), &MCP2515_u8ReceiveData);
	}

	MCP2515_voidChipSelect(SLAVE_RELEASE);
}

/**
 * @brief Initialize the MCP2515 module for receiving CAN messages.
 *
 * @note This function initializes the MCP2515 module for receiving CAN messages by configuring
 *       the SPI master, performing a reset, entering configuration mode, setting the bit timing
 *       parameters for a 250 KHz bit rate using an 8 MHz oscillator, configuring masks and filters
 *       to accept all messages, enabling the RX0 interrupt, and transitioning to the normal operating mode.
 */

void MCP2515_voidReceiveInit(void)
{
	MSPI_voidInitMaster();

	MCP2515_voidReset();

	/*Enter configuration mode and enable CLKOUT with no prescaler*/
	MCP2515_voidWriteRegister(CANCTRL, 0x84);

	while((MCP2515_u8ReadRegister(CANSTAT) >> 5) != MCP2515_MODE_CONFG);

	/*To run at 250 KHz bit rate using 8 MHz oscillator*/
	MCP2515_voidSetBitTiming((2 << 6), (1 << 7) | (6 << 3) | (1), (5));

	/*Accept all messages*/
	MCP2515_voidSetMask(RXM0SIDH, 0x00000000, 1);
	MCP2515_voidSetMask(RXM1SIDH, 0x00000000, 1);
	MCP2515_voidSetRollover(1);

	MCP2515_voidWriteRegister(CANINTE, 1<<RX0IE);

	MCP2515_voidSetMode(MCP2515_MODE_NORMAL);
}

// Array to store the received data.
uint8_t MCP2515_u8ReadBuffer[14];


/**
 * @brief Receive a CAN message from the MCP2515 module.
 *
 * @return MCP2515_CanMessage_t The received CAN message.
 *
 * @note This function receives a CAN message from the MCP2515 module by reading the contents
 *       of the RXB0 buffer. The chip select pin is controlled during the operation using
 *       MCP2515_voidChipSelect. The received message is then processed and returned as an
 *       MCP2515_CanMessage_t structure.
 */
 
MCP2515_CanMessage_t MCP2515_stReceiveCANmsg(void)
{
	MCP2515_CanMessage_t Local_stReceive = {0};
	uint8_t Local_u8Counter = 0;

	MCP2515_voidChipSelect(SLAVE_CHOOSE);

	MSPI_voidSendRecieveDataSync(INST_READ, &MCP2515_u8ReceiveData);
	MSPI_voidSendRecieveDataSync(RXB0CTRL, &MCP2515_u8ReceiveData);


	for(Local_u8Counter = 0; Local_u8Counter < 14; Local_u8Counter++)
	{
		MSPI_voidSendRecieveDataSync(0, &MCP2515_u8ReadBuffer[Local_u8Counter]);
	}

	MCP2515_voidChipSelect(SLAVE_RELEASE);

	MCP2515_voidWriteRegister(CANINTF, 0);

	Local_stReceive.id = MCP2515_u8ReadBuffer[1];
	Local_stReceive.id = Local_stReceive.id << 3;
	uint8_t Local_u8Hegazy = MCP2515_u8ReadBuffer[2];
	Local_stReceive.id |= Local_u8Hegazy >> 5;

	for(Local_u8Counter = 0; Local_u8Counter < 8; Local_u8Counter++)
	{
		Local_stReceive.data[Local_u8Counter] = MCP2515_u8ReadBuffer[6 + Local_u8Counter];
	}

	Local_stReceive.properties = GET_BIT(MCP2515_u8ReadBuffer[0],3);

	return Local_stReceive;
}

/**
 * @brief Receive a CAN message from the MCP2515 module.
 *
 * @note This function receives a CAN message from the MCP2515 module by reading the contents
 *       of the RXB0 buffer. The chip select pin is controlled during the operation using
 *       MCP2515_voidChipSelect. The received message is stored in the internal buffer
 *       MCP2515_u8ReadBuffer for further processing.
 */



