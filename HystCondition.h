#ifndef HystCondition_h
#define HystCondition_h

typedef unsigned long TIME; 
typedef unsigned int TIME_DIFF; // this allows saving a small amount of memory

class HystCondition {
  public:
	bool (*condition)();
	TIME_DIFF hystTime;
	HystCondition(TIME_DIFF hyst, bool (*cond)()) {
		start_time = 0;
		hystTime = hyst;
		condition = cond;
	}
	HystCondition(): HystCondition(-1, condNever) {}
	
	bool update(TIME time) {
		if(!(*condition)()) {
			start_time = time;
		} else if(time - start_time > hystTime) {
			return true;
		}
		return false;
	}
	inline void reset(TIME time) { start_time = time; }
	
  private:
	TIME start_time;
	static bool condNever() { return false; }
};

#endif