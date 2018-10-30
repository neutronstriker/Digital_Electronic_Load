/* 
* fifo.cpp
*
* Created: 07-05-2016 01:01:22
* Author: neutron
*/

/************************************************************************/
/* 

Dt:22-05-2016:
I have recently learned another method of enque and deque but it it 
not optimal in memory usage as it will be able to use 1 index size less than
the specified.

void enque(data)
{
	if ((tail+1)%length == head)
	{
		return;	//buffer is full
	}
	else
	{
		buffer[tail] = data;
		tail = (tail+1)%length;
	}
}

int deque()
{
	int val;
	if(head == tail)
	{
		return 0;	//buffer is empty
	}
	else
	{
		val = buffer[head];
		head = (head+1)%length;
	}
}

in both the functions above if you are worried about using modulus operator then you can use
these statements:

tail++;
if(tail == length-1)
tail=0;

and when the modulus based operations are replaced by the above three statements in both functions
then for checking of queue full situation the condition simply becomes if((head==0 && tail == length-1)|| (tail+1 == head)).

similarly we can repeat for head also.

But in this case lets say I make a buffer of size 10 but I will be always be able to store only 9 indexes.

But the advantage i have because of this is speed improvement, not using int type of head and tail and code simplicity.

So if we have types like uint8_t for buffer then we can use this as it will improve the speed, and even if there will
be memory loss because of un-utilization of allocated memory in structure types if we want speed we can certainly go
for this.        */
/************************************************************************/


#include "fifo.h"

uint8_t fifo::isEmpty()
{
	if(head == -1 && tail == -1)
	{
		return 1; //buffer is empty, nothing to do.
	}
	else
	{
		return 0;
	}	
}

uint8_t fifo::isFull()
{
	if((head==0 && tail == length-1)|| (tail == head-1))
	{
		return 1;		//buffer is full
	}
	else
	{
		return 0;
	}
}

fifo_word::fifo_word( int size )
{
	//this->init(size);
	init(size);
}

void fifo_word::init( int size )
{
	buffer = new uint16_t(size);
	length=size;
	head=tail=-1;
}

uint16_t fifo_word::deque()
{
	uint16_t val;

	if(head == -1 && tail == -1)
	{
		return 0; //buffer is empty, nothing to do.
	}
	
	else if(head==tail)
	{
		val=buffer[head];
		head=tail=-1;
	}
	else if(tail < head && head==length-1)
	{
		val = buffer[head];
		head=0;
	}
	else
	{
		val = buffer[head];
		head++;

	}
	return val;
}

void fifo_word::enque( uint16_t data )
{
	if((head==0 && tail == length-1)|| (tail == head-1))
	{
		return;		//buffer is full, can't insert.
	}

	else if(tail==length-1 && head > 0)
	{
		tail=0;
	}

	else
	{
		tail++;
	}

	//this->buffer[tail]=data;
	buffer[tail]=data;

	if(head==-1)
	head=0;	
}

 fifo_word::~fifo_word()
{
	//delete [] buffer;
}




fifo_custom_type::fifo_custom_type( int size )
{
	init(size);	
}

 fifo_custom_type::~fifo_custom_type()
{
	
}

void fifo_custom_type::init( int size )
{
	buffer = new custom_buffer_type[size];		//when it is custom type specified by struct/enum you can't use new <type>(size)
	length=size;								//you have to use new <type>[size]
	head=tail=-1;
}

void fifo_custom_type::enque( custom_buffer_type data )
{
	if((head==0 && tail == length-1)|| (tail == head-1))
	{
		return;		//buffer is full, can't insert.
	}

	else if(tail==length-1 && head > 0)
	{
		tail=0;
	}

	else
	{
		tail++;
	}

	//this->buffer[tail]=data;
	buffer[tail]=data;

	if(head==-1)
	head=0;
}

custom_buffer_type fifo_custom_type::deque()
{
	custom_buffer_type val;

	if(head == -1 && tail == -1)
	{
		return val; //In custom type case always use isEmpty / isFull functions, don't rely on this return val, this will be garbage.
	}
	
	else if(head==tail)
	{
		val=buffer[head];
		head=tail=-1;
	}
	else if(tail < head && head==length-1)
	{
		val = buffer[head];
		head=0;
	}
	else
	{
		val = buffer[head];
		head++;

	}
	return val;
}



fifo_byte::fifo_byte( int size )
{
	//this->init(size);
	init(size);
}

void fifo_byte::init( int size )
{
	buffer = new uint8_t(size);
	length=size;
	head=tail=-1;
}

uint8_t fifo_byte::deque()
{
	uint8_t val;

	if(head == -1 && tail == -1)
	{
		return 0; //buffer is empty, nothing to do.
	}
	
	else if(head==tail)
	{
		val=buffer[head];
		head=tail=-1;
	}
	else if(tail < head && head==length-1)
	{
		val = buffer[head];
		head=0;
	}
	else
	{
		val = buffer[head];
		head++;

	}
	return val;
}

void fifo_byte::enque( uint8_t data )
{
	if((head==0 && tail == length-1)|| (tail == head-1))
	{
		return;		//buffer is full, can't insert.
	}

	else if(tail==length-1 && head > 0)
	{
		tail=0;
	}

	else
	{
		tail++;
	}

	//this->buffer[tail]=data;
	buffer[tail]=data;

	if(head==-1)
	head=0;
}

fifo_byte::~fifo_byte()
{
	//delete [] buffer;
}








