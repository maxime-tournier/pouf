#ifndef TRIANGLE_MODEL_H
#define TRIANGLE_MODEL_H

#include <sofa/component/collision/TriangleModel.h>

template<class T>
struct triangle_model : sofa::component::collision::TTriangleModel<T> {
	typedef sofa::component::collision::TTriangleModel<T> base;

	
    virtual bool canCollideWith(sofa::core::CollisionModel* model)
    {
		// f*ck you type system !
		sofa::Data< sofa::core::objectmodel::TagSet > sofa::core::CollisionModel::* ptr =
			&triangle_model::collisionGroupTags;


        if (model->getContext() == this->getContext())
            return this->bSelfCollision.getValue();
        else if(!(this->collisionGroupTags.getValue().empty())) {
            if( (model->*ptr).getValue().empty())
                return true;
			
            sofa::core::objectmodel::TagSet::const_iterator it = this->collisionGroupTags.getValue().begin();
            for(;it != this->collisionGroupTags.getValue().end() ; ++it) {
				
				if( (model->*ptr).getValue().includes(*it)) {
					return false;
				}
			}
            return true;
        }
        else
            return true;
    }

};

#endif
