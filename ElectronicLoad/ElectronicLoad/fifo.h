/* 
* fifo.h
*
* Created: 07-05-2016 01:01:22
* Author: neutron
*/


#ifndef __FIFO_H__
#define __FIFO_H__

#include <stdint.h>


#include "config.h"





//typedef float custom_buffer_type;
struct custom_buffer_type{		//this is there so that whenever we want we can just change the typedef to whatever we 
										//want not just struct, it can be any known type like long(int32_t) or double etc.
										//or any other existing object class or a union etc.
										//so for know only byte, word and custom three types have been implemented
										//if required either you can have a custom union type or typedef custom to float.
};



class fifo{

protected:
	int head;
	int tail;
	int length;
public:
		
	virtual void init(int size)=0;	//only this function will have same parameter type through all derived classes
	
	uint8_t isEmpty();		//this implementation will be same in all derived classes so will be declared here.
	uint8_t isFull();
	
	fifo(){
		//do nothing let the child class do initialization
	}
	
	~fifo(){
		
	}

};

class fifo_word : public fifo{

private:
	uint16_t *buffer;

public:
	fifo_word();
	fifo_word(int size);
	~fifo_word();
	void init(int size);
	uint16_t deque();
	void enque(uint16_t data);
	
};


class fifo_byte : public fifo{
	
		private:
		uint8_t *buffer;

		public:
		fifo_byte();
		fifo_byte(int size);
		~fifo_byte();
		void init(int size);
		uint8_t deque();
		void enque(uint8_t data);
	
}; 

	

class fifo_custom_type:public fifo{


private:
	custom_buffer_type *buffer;
	
public:
	fifo_custom_type();
	fifo_custom_type(int size);
	~fifo_custom_type();
	void init(int size);
	custom_buffer_type deque();
	void enque(custom_buffer_type data);
	
	
};











#endif //__FIFO_H__
