/*
	ILI9163C - A fast SPI driver for TFT that use Ilitek ILI9163C.
	
	Features:
	- Very FAST!, expecially with Teensy 3.x where uses DMA SPI.
	- It uses just 4 or 5 wires.
	- Compatible at command level with Adafruit display series so it's easy to adapt existing code.
	- It uses the standard Adafruit_GFX Library (you need to install). 
	
	Background:
	I got one of those displays from a chinese ebay seller but unfortunatly I cannot get
	any working library so I decided to hack it. ILI9163C looks pretty similar to other 
	display driver but it uses it's own commands so it's tricky to work with it unlsess you
	carefully fight with his gigantic and not so clever datasheet.
	My display it's a 1.44"", 128x128 that suppose to substitute Nokia 5110 LCD and here's the 
	first confusion! Many sellers claim that it's compatible with Nokia 5110 (that use a philips
	controller) but the only similarity it's the pin names since that this one it's color and
	have totally different controller that's not compatible.
	http://www.ebay.com/itm/Replace-Nokia-5110-LCD-1-44-Red-Serial-128X128-SPI-Color-TFT-LCD-Display-Module-/141196897388
	http://www.elecrow.com/144-128x-128-tft-lcd-with-spi-interface-p-855.html
	Pay attention that   can drive different resolutions and your display can be
	160*128 or whatever, also there's a strain of this display with a black PCB that a friend of mine
	got some weeks ago and need some small changes in library to get working.
	If you look at TFT_ILI9163C.h file you can add your modifications and let me know so I
	can include for future versions.
	
	Code Optimizations:
	The purpose of this library it's SPEED. I have tried to use hardware optimized calls
	where was possible and results are quite good for most applications, actually nly filled circles
    are still a bit slow. Many SPI call has been optimized by reduce un-needed triggers to RS and CS
	lines. Of course it can be improved so feel free to add suggestions.
	-------------------------------------------------------------------------------
    Copyright (c) 2014, .S.U.M.O.T.O.Y., coded by Max MC Costa.    

    TFT_ILI9163C Library is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    TFT_ILI9163C Library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    This file needs the following Libraries:
 
    Adafruit_GFX by Adafruit:
    https://github.com/adafruit/Adafruit-GFX-Library
	Remember to update GFX library often to have more features with this library!
	From this version I'm using my version of Adafruit_GFX library:
	https://github.com/sumotoy/Adafruit-GFX-Library
	It has faster char rendering and some small little optimizations but you can
	choose one of the two freely since are both fully compatible.
	''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
	Special Thanks:
	Thanks Adafruit for his Adafruit_GFX!
	Thanks to Paul Stoffregen for his beautiful Teensy3 and DMA SPI.
	
	+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	Version:
	0.1a1: First release, compile correctly. Altrough not fully working!
	0.1a3: Better but still some addressing problems.
	0.1b1: Beta! Addressing solved, now rotation works and boundaries ok.
	0.2b1: Cleaned up.
	0.2b3: Added 2.2" Red PCB parameters
	0.2b4: Bug fixes, added colorSpace (for future send image)
	0.2b5: Cleaning
	0.3b1: Complete rework on Teensy SPI based on Paul Stoffregen work
	SPI transaction,added BLACK TAG 2.2 display
	0.3b2: Minor fix, load 24bit image, Added conversion utility
	0.4:	some improvement, new ballistic gauge example!
	0.5:	Added scroll and more commands, optimizations
	Fixed a nasty bug in fill screen!
	+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	BugList of the current version:
	
	Please report any!

	
Here's the speed test between 0.2b5 and 0.3b1 on Teensy3.1 (major SPI changes)
------------------------------------------------------------------------
Lines                    17024  	16115	BETTER
Horiz/Vert Lines         5360		5080	BETTER
Rectangles (outline)     4384		4217	BETTER
Rectangles (filled)      96315		91265	BETTER
Circles (filled)         16053		15829	LITTLE BETTER
Circles (outline)        11540		20320	WORST!
Triangles (outline)      5359		5143	BETTER
Triangles (filled)       19088		18741	BETTER
Rounded rects (outline)  8681		12498	LITTLE WORST
Rounded rects (filled)   105453		100213	BETTER
Done!


*/

/*
Probable RED PCB config, bottom aligned? Can this happen???

128 cols x 160 rows, GM[2:0] = "011"

ILI9163C    LCD panel
---------------------
Gate2       N.C.
Gate3       N.C.
  :           :
Gate33      N.C.
Gate34      Gate1
Gate35      Gate2
  :           :
Gate160     Gate127
Gate161     Gate128

*/

/**
 * TFT_ILI9163C library for ST Nucleo F411RE
 * 
 * @author Copyright (c) 2014, .S.U.M.O.T.O.Y., coded by Max MC Costa
 * https://github.com/sumotoy/TFT_ILI9163C
 *
 * @author modified by masuda, Masuda Naika
 */
 
#ifndef _TFT_ILI9163CLIB_H_
#define _TFT_ILI9163CLIB_H_

#if defined(TARGET_NUCLEO_F411RE)
//    #define __F411RE_SOFT__
//   #define __F411RE_DMA__
  #define __F411RE_DMA_IT__
#elif defined(TARGET_NUCLEO_F302R8)
//    #define __F302R8_SOFT__
    #define __F302R8_DMA__
#else
    #define __MBED_GENERIC__
#endif


#if defined(__F411RE_SOFT__)
    #include "TFT_ILI9163C_F411RE_SOFT.h"
#elif defined(__F411RE_DMA__)
    #include "TFT_ILI9163C_F411RE_DMA.h"
#elif defined(__F411RE_DMA_IT__)
    #include "TFT_ILI9163C_F411RE_DMA_IT.h"
#elif defined(__F302R8_SOFT__)
    #include "TFT_ILI9163C_F302R8_SOFT.h"
#elif defined(__F302R8_DMA__)
    #include "TFT_ILI9163C_F302R8_DMA.h"
#else
    #include "TFT_ILI9163C_GENERIC.h"
#endif

#endif
