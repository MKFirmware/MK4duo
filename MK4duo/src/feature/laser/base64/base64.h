/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 Alberto Cotronei @MagoKimbra
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * Copyright (c) 2013 Adam Rudd.
 * See LICENSE for more information
 */

#ifndef _BASE64_H
#define _BASE64_H

#if ENABLED(LASER) && ENABLED(LASER_RASTER)

  /* b64_alphabet:
   * 		Description: Base64 alphabet table, a mapping between integers
   * 					 and base64 digits
   * 		Notes: This is an extern here but is defined in Base64.c
   */
  extern const char b64_alphabet[];

  /* base64_encode:
   * 		Description:
   * 			Encode a string of characters as base64
   * 		Parameters:
   * 			output: the output buffer for the encoding, stores the encoded string
   * 			input: the input buffer for the encoding, stores the binary to be encoded
   * 			inputLen: the length of the input buffer, in bytes
   * 		Return value:
   * 			Returns the length of the encoded string
   * 		Requirements:
   * 			1. output must not be null or empty
   * 			2. input must not be null
   * 			3. inputLen must be greater than or equal to 0
   */
  int base64_encode(char *output, char *input, int inputLen);

  /* base64_decode:
   * 		Description:
   * 			Decode a base64 encoded string into bytes
   * 		Parameters:
   * 			output: the output buffer for the decoding,
   * 					stores the decoded binary
   * 			input: the input buffer for the decoding,
   * 				   stores the base64 string to be decoded
   * 			inputLen: the length of the input buffer, in bytes
   * 		Return value:
   * 			Returns the length of the decoded string
   * 		Requirements:
   * 			1. output must not be null or empty
   * 			2. input must not be null
   * 			3. inputLen must be greater than or equal to 0
   */
  int base64_decode(unsigned char *output, char *input, int inputLen);

  /* base64_enc_len:
   * 		Description:
   * 			Returns the length of a base64 encoded string whose decoded
   * 			form is inputLen bytes long
   * 		Parameters:
   * 			inputLen: the length of the decoded string
   * 		Return value:
   * 			The length of a base64 encoded string whose decoded form
   * 			is inputLen bytes long
   * 		Requirements:
   * 			None
   */
  int base64_enc_len(int inputLen);

  /* base64_dec_len:
   * 		Description:
   * 			Returns the length of the decoded form of a
   * 			base64 encoded string
   * 		Parameters:
   * 			input: the base64 encoded string to be measured
   * 			inputLen: the length of the base64 encoded string
   * 		Return value:
   * 			Returns the length of the decoded form of a
   * 			base64 encoded string
   * 		Requirements:
   * 			1. input must not be null
   * 			2. input must be greater than or equal to zero
   */
  int base64_dec_len(char *input, int inputLen);

#endif // ENABLED(LASER) && ENABLED(LASER_RASTER)

#endif /* _BASE64_H */
