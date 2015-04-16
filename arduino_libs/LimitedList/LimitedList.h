#ifndef _LIMITEDLIST_H
#define _LIMITEDLIST_H

// the definition of the queue class.
template<typename T, int N>
class LimitedList {
	protected:
		T queue[N];
		int _count;
		bool new_is_priority;
	public:
		LimitedList(bool new_is_priority=true):_count(0),new_is_priority(new_is_priority) {}

		bool push(const T v)
		{
			bool rs = new_is_priority || _count < N-1;
			if(rs) {
				queue[_count++] = v;
				if(_count >= N) {
					for(int i=1; i<N; i++) {
						queue[i-1] = queue[i];
					}
					_count = N - 1;
				}
			}
			return rs;
		}

		inline void clear()
		{
			_count = 0;
		}

		inline int count()
		{
			return _count;
		}

		T get(int index)
		{
			return (index>=0 && index<_count) ? queue[index] : T();
		}

		void remove(int index) {
			if(index < _count) {
				_count--;
				for(int i=index; i<_count; i++) {
					queue[i] = queue[i+1];
				}
				queue[_count] = T();
			}
		}
};

template<typename T, int N>
class ScalarLimitedList: public LimitedList<T, N> {

	public:
		T vmax(T initval)
		{
			T rs = initval;
			for(int i=0; i<this->_count; i++) {
				if(rs < this->queue[i]) {
					rs = this->queue[i];
				}
			}
			return rs;
		}

		T vmin(T initval)
		{
			T rs = initval;
			for(int i=0; i<this->_count; i++) {
				if(rs > this->queue[i]) {
					rs = this->queue[i];
				}
			}
			return rs;
		}

		T average(T initval)
		{
			T rs = initval;
			if(this->_count > 0) {
				for(int i=0; i<this->_count; i++) {
					rs += this->queue[i];
				}
				rs /= this->_count;
			}
			return rs;
		}

};

#endif //_LIMITEDLIST_H

