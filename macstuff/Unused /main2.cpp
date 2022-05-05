//
//  main.cpp
//  rtl
//
//  Created by Vincent Moscaritolo on 4/18/22.
//

#include <stdio.h>
#include <stdlib.h>   // exit()
#include <unistd.h>

#include <stdexcept>
#include "CommonDefs.hpp"

#include "CAR_Radio.hpp"

// ./rtl  | play -r 32k -t raw -e s -b 16 -c 1 -q --no-show-progress -V1 -

int main(int argc, const char * argv[]) {
	
	
	try {
		CAR_Radio radio;
	
		radio.setMode(CAR_Radio::MODE_WBFM);
		radio.setFrequency("88.3M");
		
		if( radio.begin()) {
			
			
			while (true) {
				usleep(10000000);
				radio.setFrequency("106.7M");
				usleep(10000000);
				radio.setFrequency("88.3M");

				
			}

		}

		radio.stop();
		
	}
		catch ( const Exception& e)  {
			printf("\tError %d %s\n\n", e.getErrorNumber(), e.what());
			return -1;
		}
		catch (std::invalid_argument& e)
		{
			printf("EXCEPTION: %s ",e.what() );
			return -1;
		}

		return EXIT_SUCCESS;
	
}
