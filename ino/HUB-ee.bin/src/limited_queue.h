#ifndef _LIMITEDQUEUE_H
#define _LIMITEDQUEUE_H

// the definition of the queue class.
template<typename T, int N>
class LimitedQueue {
	protected:
		T queue[N];
		int pointer;
	public:
		LimitedQueue():pointer(0) {}

		void push(const T v)
		{
			queue[pointer++] = v;
			if(pointer >= N) {
				for(int i=1; i<N; i++) {
					queue[i-1] = queue[i];
				}
				pointer = N - 1;
			}
		}

		inline void clear()
		{
			pointer = 0;
		}

		inline int count()
		{
			return pointer;
		}

		T get(int index)
		{
			return (index>=0 && index<pointer) ? queue[index] : T();
		}

};

template<typename T, int N>
class ScalarLimitedQueue: public LimitedQueue<T, N> {

	public:
		T vmax(T initval)
		{
			T rs = initval;
			for(int i=0; i<this->pointer; i++) {
				if(rs < this->queue[i]) {
					rs = this->queue[i];
				}
			}
			return rs;
		}

		T vmin(T initval)
		{
			T rs = initval;
			for(int i=0; i<this->pointer; i++) {
				if(rs > this->queue[i]) {
					rs = this->queue[i];
				}
			}
			return rs;
		}

		T average(T initval)
		{
			T rs = initval;
			if(this->pointer > 0) {
				for(int i=0; i<this->pointer; i++) {
					rs += this->queue[i];
				}
				rs /= this->pointer;
			}
			return rs;
		}

};

#endif //_LIMITEDQUEUE_H

