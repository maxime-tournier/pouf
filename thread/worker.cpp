#include "worker.h"

#include <iostream>

namespace thread {


  struct exit { };
  

  worker::worker() {
	lock_type lock(mutex);
	thread = std::thread( [&] { this->loop(); } );

	// wait for thread to notify us
	cv.wait(lock);

	// lock is reacquired so we don't exit until thread is waiting on
	// cv
  }


  void worker::loop() {
	
	// block until constructor is waiting on us
	lock_type lock(mutex);

	// notify constructor we started
	cv.notify_one();

	try {
	  
	  bool first = true;

	  task_type task;
	  while( true ) {
		{
		  lock_type sub = first ? std::move(lock) : lock_type(mutex);
		  cv.wait(sub, [&] { return !queue.empty();});
		  task = std::move( queue.front() );
		  queue.pop();
		}

		first = false;
		task();
	  }
	  
	}
	catch( exit e) { }
  }

  void worker::push(const task_type& task) {
	{
	  lock_type lock(mutex);
	  queue.push( task );
	}
	cv.notify_one();
  }

  void worker::push(task_type&& task) {
	{
	  lock_type lock(mutex);
	  queue.push( std::move(task) );
	}
	cv.notify_one();
  }


  void worker::unsafe_push(const task_type& task) {
	queue.push( task );
	cv.notify_one();
  }


  worker::~worker() {

	push([&] { throw exit(); });
	thread.join();
	
  }

}
