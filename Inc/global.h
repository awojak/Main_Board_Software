/*
 * global.h
 *
 *  Created on: 19.02.2020
 *      Author: Neo
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#define TRUE 1
#define FALSE 0

/*! \brief Status flags
 */
struct GLOBAL_FLAGS {
  //! True when stepper motor is running.
  unsigned char running:1;
  //! True when uart has received a string (ended with '/r').
  unsigned char cmd:1;
  //! Dummy bits to fill up a byte.
  unsigned char dummy:6;
};

//! Global status flags
struct GLOBAL_FLAGS status;

#endif /* GLOBAL_H_ */
