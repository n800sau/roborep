#ifndef _LIMITEDPTRLIST_H
#define _LIMITEDPTRLIST_H

// the definition of the list of pointers class.
template<typename T, int N>
class LimitedPtrList {
	protected:
		T *queue[N];
		int _count;
		bool new_is_priority;
	public:
		LimitedPtrList(bool new_is_priority=true):_count(0),new_is_priority(new_is_priority) {}

		bool push(T *v)
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

		inline void clear(bool destroy=true)
		{
			if(destroy) {
				for(int i=0; i< _count; i++) {
					delete queue[i];
				}
			}
			_count = 0;
		}

		inline int count()
		{
			return _count;
		}

		T *get(int index)
		{
			return (index>=0 && index<_count) ? queue[index] : NULL;
		}

		void remove(int index, bool destroy=true) {
			if(index < _count) {
				if(destroy) {
					delete queue[index];
				}
				_count--;
				for(int i=index; i<_count; i++) {
					queue[i] = queue[i+1];
				}
			}
		}

};

#endif //_LIMITEDPTRLIST_H
