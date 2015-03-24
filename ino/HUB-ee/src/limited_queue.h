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

		T vmax(T initval)
		{
			T rs = initval;
			for(int i=0; i<pointer; i++) {
				if(rs < queue[i]) {
					rs = queue[i];
				}
			}
			return rs;
		}

		T vmin(T initval)
		{
			T rs = initval;
			for(int i=0; i<pointer; i++) {
				if(rs > queue[i]) {
					rs = queue[i];
				}
			}
			return rs;
		}

		T average(T initval)
		{
			T rs = initval;
			if(pointer > 0) {
				for(int i=0; i<pointer; i++) {
					rs += queue[i];
				}
				rs /= pointer;
			}
			return rs;
		}
};

#endif //_LIMITEDQUEUE_H

