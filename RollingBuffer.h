#ifndef RollingBuffer_h
#define RollingBuffer_h

#include <stdlib.h>

template <class T, size_t size>
class RollingBuffer {
  public:
	bool push(const T& item) {
		if(isFull()) {
			return false; // buffer full
		}
		data[pushIndex] = item;
		pushIndex++;
		pushIndex %= size;
		empty = false;
		return true;
	}
	
	bool pop(T* item) {
		if(empty) {
			return false;
		}
		*item = data[popIndex];
		popIndex++;
		popIndex %= size;
		if(pushIndex == popIndex) {
			empty = true;
		}
		return true;
	}
	
	inline bool isEmpty() const { return empty; }
	inline bool isFull() const { return !empty && pushIndex == popIndex; }
	
	void clear() {
		pushIndex = 0;
		popIndex = 0;
		empty = true;
	}
	
  private:
	bool empty = true;
	T data[size];
	size_t pushIndex = 0;
	size_t popIndex = 0;
};

#endif