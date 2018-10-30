/*

void loop()
{
	 //ISR method works lot better
	if(pc0 > pc2)
	{
		lcd.clearDisplay();
		lcd.setTextColor(WHITE);
		lcd.setTextSize(2);
		lcd.setCursor(0,0);
		lcd.print("clock");
		lcd.display();
		state=0;
	}
	else if(pc0 < pc2)
	{
		lcd.clearDisplay();
		lcd.setTextColor(WHITE);
		lcd.setTextSize(2);
		lcd.setCursor(0,0);
		lcd.print("anti-clock");
		lcd.display();
		state=1;
	}
	
	else
	{
		if(state==1)
		{
			lcd.clearDisplay();
			lcd.setTextColor(WHITE);
			lcd.setTextSize(2);
			lcd.setCursor(0,0);
			lcd.print("clock");
			lcd.display();
		}
		else
		{
			lcd.clearDisplay();
			lcd.setTextColor(WHITE);
			lcd.setTextSize(2);
			lcd.setCursor(0,0);
			lcd.print("anti-clock");	
			lcd.display();
		}
	}
	
		
	
	
	data = PINB & 0x03;
	if(state != data)
	{
		if(data == 0)
		{
			if(state == 1)
			{
				anti();
			}
		}
		else if (data == 1)
		{
			if (state == 0)
			{
				clockwise();
			}
				
		}
		else if (data == 2)
		{
			if (state == 3)
			{
				clockwise();
			}
			
		}
		else if (data == 3)
		{
			if (state == 2)
			{
				anti();
			}
			
		}
		
						//try adding low pass filter to reduce glitch.	
		state = data;
		
		//delay(30); //makes it go mad, try low pass.
		//and then instead of polling which may be affected by delay we will put this in one of the PCINT
		//then we will use a small fifo to put the values of clock and anti clock in it where clock means 0 and 
		//anti-clock will be 1, the buffer will be 16byte and will be controlled by circular fifo construct.
		
	}
	
 

}
 * old_testing_code_log.c
 *
 * Created: 07-05-2016 00:53:57
 *  Author: neutron
 */ 
/*
#define BUF_TYPE_INT	0
#define BUF_TYPE_UINT	1
#define BUF_TYPE_BYTE	2
#define BUF_TYPE_FLOAT	3
#define BUF_TYPE_BOOL	4

union buffer_data_type{
	char char_type;
	uint8_t byte_type;
	int	int_type;
	uint16_t uint16_type;
	float	float_type;
	};

class fifo{

private:
	int *buffer_int;
	unsigned char *buffer_byte;
	unsigned int	*buffer_word;
	bool	*buffer_bool;
	float *buffer_float;
	int8_t head;
	int8_t tail;
	int length;
	unsigned char buffer_type;
public:
	void enque(int data);
	int deque_int();
	unsigned int deque_uint();
	
	void init(int size, unsigned char type);
	fifo();
	fifo(int size,unsigned char type); //since we will be using new operator may be we should not call init with in constructor, especially when
	~fifo(); //the class object will be global.

};
*/

/*
void fifo::init(int size,unsigned char type)
{
	if(size<=0)
	{
		return;
	}
	switch(type)
	{
		case BUF_TYPE_INT:
			buffer_type = BUF_TYPE_INT;
			break;
		case BUF_TYPE_UINT:
			buffer_type = BUF_TYPE_UINT;
			break;
		case BUF_TYPE_BYTE:
			buffer_type = BUF_TYPE_BYTE;
			break;
		case BUF_TYPE_FLOAT:
			buffer_type = BUF_TYPE_FLOAT;
			break;
		//case BUF_TYPE_BOOL:
		default:
			buffer_type = BUF_TYPE_INT;
			break;
	}
	buffer = new int(size);
	length=size;
	head=tail=-1;
}

void fifo::enque(int data)
{
	if((head==0 && tail == length-1)|| (tail == head-1))
	{
		//cout<<"que is full"<<endl;
		return;
	}

	else if(tail==length-1 && head > 0)
	{
		tail=0;
	}

	else
	{
		tail++;
	}

	buffer[tail]=data;

	if(head==-1)
	head=0;
}

int fifo::deque()
{
	int val;

	if(head == -1 && tail == -1)
	{
		//cout<<"que is empty"<<endl;
		return 0; //buffer is empty, nothing to do.
	}
	else if(head==tail)
	{
		val=buffer[head];
		buffer[head]=0;//extra code for debugging
		head=tail=-1;
	}
	else if(tail < head && head==length-1)
	{
		val = buffer[head];
		buffer[head]=0;//extra code for debugging
		head=0;
	}
	else
	{
		val = buffer[head];
		buffer[head]=0;//extra code for debugging
		head++;

	}
	return val;
}

fifo::fifo(int size)
{
	init(size);
}

fifo::~fifo()
{
	//destructor code
	//delete [] buffer; //anyhow this will be done automatically but still doing it.
}

*/

//well template works but only for constants in here, so we could try that but not now.


/*
void anti()
{
	static uint16_t count = 0;
	count++;
	
	lcd.clearDisplay();
	lcd.setTextColor(WHITE);
	lcd.setTextSize(2);
	lcd.setCursor(0,0);
	lcd.print("anti-clock");
	
	lcd.print(count);
	
	lcd.display();
}

void clockwise()
{
	static uint16_t count = 0;
	count++;
	
	lcd.clearDisplay();
	lcd.setTextColor(WHITE);
	lcd.setTextSize(2);
	lcd.setCursor(0,0);
	lcd.println("clock");
	
	lcd.print(count);
	
	lcd.display();
}


void pot_increase(uint8_t channel)
{
	pot_channel_val[channel]++;
	i2c_write(TPL0102_ADDRESS,channel,&pot_channel_val[channel],1);
		
}

void pot_decrease(uint8_t channel)
{
	
	pot_channel_val[channel]--;
	i2c_write(TPL0102_ADDRESS,channel,&pot_channel_val[channel],1);
		
}

*/
