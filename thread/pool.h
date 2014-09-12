#ifndef THREAD_POOL_H
#define THREAD_POOL_H

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

	// note: the calling thread will also work
	template<class Int, class F>
	void parallel_for(Int start, Int end, const F& f) const {

	  std::atomic<unsigned> ok(0);
	  
	  const unsigned n = worker.size() + 1;
	  
	  auto make_task = [&](unsigned i) {
		const Int delta = end - start;

		const Int s = start + (delta * i) / n;
		const Int e = start + (delta * (i + 1)) / n;
		
		return [s, e, &ok, &f] {
		  f(s, e);
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



#endif
