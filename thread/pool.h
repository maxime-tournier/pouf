#ifndef THREAD_POOL_H
#define THREAD_POOL_H

#ifdef SOFA_HAVE_STD_THREAD
#include "worker.h"


#include <list>

namespace thread {

  // a simple thread pool for parallel for loops
  struct pool {

	pool(unsigned n = std::thread::hardware_concurrency() - 1) {
	  for(unsigned i = 0; i < n; ++i) {
		worker.emplace_back( new thread::worker );
	  }
	}
	
	std::vector< std::unique_ptr<thread::worker> > worker;

	unsigned size() const { return 1 + worker.size(); }
	
	struct chunk {
	  int start, end;
	  unsigned id;
	};
	
	// note: the calling thread will also work
	template<class F>
	void parallel_for(int start, int end, const F& f) const {

	  std::atomic<unsigned> ok(0);
	  
	  const unsigned n = size();
	  
	  auto make_task = [&](unsigned i) {
		const int delta = end - start;

		const int s = start + (delta * i) / n;
		const int e = start + (delta * (i + 1)) / n;

		chunk c;

		c.start = s;
		c.end = e;
		c.id = i;
		
		return [c, &ok, &f] {
		  f(c);
		  ++ok;
		};
	  };

	  // push tasks on workers
	  for(unsigned i = 0; i < n - 1; ++i) {
		worker[i]->push( make_task(i + 1) );
	  }

	  // this thread takes last task
	  make_task(0)();
	  
	  // spinlock 
	  while( ok != n ) { }
	};
	
  };



}


#else

namespace thread {

  struct pool {

	pool(unsigned n = 0) {
	  if(n) throw std::logic_error("you need c++11 <thread> support to get multiple threads");
	}
	
	template<class F>
	void parallel_for(int start, int end, const F& f) const {
	  f( start, end );
	}
	
  };
}

#endif



#endif
