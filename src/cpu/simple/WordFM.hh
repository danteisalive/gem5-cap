/*
 * Copyright (c) 2011-2012,2015 ARM Limited
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Steve Reinhardt
 *          Dave Greene
 *          Nathan Binkert
 */

#ifndef __CPU_SIMPLE_WORDFM_HH__
#define __CPU_SIMPLE_WORDFM_HH__

#include <cassert>
#include <cstring>
#include <iostream>
#include <string>

// Size in bits on:                          32-bit archs   64-bit archs
//                                           ------------   ------------
typedef unsigned long          UWord;     // 32             64
typedef   signed long           Word;     // 32             64

/* Always 8 bits. */
typedef  unsigned char   UChar;
typedef    signed char   Char;
typedef           char   HChar; /* signfulness depends on host */
                                /* Only to be used for printf etc
                                   format strings */

/* Always 16 bits. */
typedef  unsigned short  UShort;
typedef    signed short  Short;

/* Always 32 bits. */
typedef  unsigned int    UInt;
typedef    signed int    Int;

/* Always 64 bits. */
typedef  unsigned long long int   ULong;
typedef    signed long long int   Long;

/* Equivalent of C's size_t type. The type is unsigned and has this
   storage requirement:
   32 bits on a 32-bit architecture
   64 bits on a 64-bit architecture. */
typedef  unsigned long SizeT;

/* Bool is always 8 bits. */
typedef  unsigned char  Bool;
#define  True   ((Bool)1)
#define  False  ((Bool)0)

/* An address: 32-bit or 64-bit wide depending on host architecture */
typedef unsigned long Addr;

/* Tracks information about live blocks. */
typedef
   struct {
      Addr        payload;
      SizeT       req_szB;
      Addr        pid;
      ULong       n_reads;
      ULong       n_writes;
   }
   Block;
//------------------------------------------------------------------//
//---                           WordFM                           ---//
//---                       Implementation                       ---//
//------------------------------------------------------------------//

/* One element of the AVL tree */
typedef
   struct _AvlNode {
      UWord key;
      UWord val;
      struct _AvlNode* child[2]; /* [0] is left subtree, [1] is right */
      Char balance; /* do not make this unsigned */
   }
   AvlNode;

typedef
   struct {
      UWord w;
      Bool b;
   }
   MaybeWord;

#define WFM_STKMAX    32    // At most 2**32 entries can be iterated over

struct _WordFM {
   AvlNode* root;
   Word     (*kCmp)(UWord,UWord);
   AvlNode* nodeStack[WFM_STKMAX]; // Iterator node stack
   Int      numStack[WFM_STKMAX];  // Iterator num stack
   Int      stackTop;              // Iterator stack pointer, one past end
};

typedef  struct _WordFM  WordFM; /* opaque */



/* Swing to the left.  Warning: no balance maintainance. */
void avl_swl ( AvlNode** root );

/* Swing to the right.  Warning: no balance maintainance. */
void avl_swr ( AvlNode** root );

/* Balance maintainance after especially nasty swings. */
void avl_nasty ( AvlNode* root );

/* Find size of a non-NULL tree. */
UWord size_avl_nonNull ( AvlNode* nd );
/* Unsignedly compare w1 and w2.  If w1 < w2, produce a negative
   number; if w1 > w2 produce a positive number, and if w1 == w2
   produce zero. */
Word cmp_unsigned_Words ( UWord w1, UWord w2 );

/* Insert element a into the AVL tree t.  Returns True if the depth of
   the tree has grown.  If element with that key is already present,
   just copy a->val to existing node, first returning old ->val field
   of existing node in *oldV, so that the caller can finalize it
   however it wants.
*/

Bool avl_insert_wrk ( AvlNode**         rootp,
                      /*OUT*/MaybeWord* oldV,
                      AvlNode*          a,
                      Word              (*kCmp)(UWord,UWord) );


/* Remove an element a from the AVL tree t.  a must be part of
   the tree.  Returns True if the depth of the tree has shrunk.
*/

Bool avl_remove_wrk ( AvlNode** rootp,
                      AvlNode*  a,
                      Word(*kCmp)(UWord,UWord) );

/* Remove the root of the AVL tree *rootp.
 * Warning: dumps core if *rootp is empty
 */

Bool avl_removeroot_wrk ( AvlNode** rootp,
                          Word(*kCmp)(UWord,UWord) );


AvlNode* avl_find_node ( AvlNode* t, Word k, Word(*kCmp)(UWord,UWord) );


Bool avl_find_bounds ( AvlNode* t,
                       /*OUT*/UWord* kMinP, /*OUT*/UWord* vMinP,
                       /*OUT*/UWord* kMaxP, /*OUT*/UWord* vMaxP,
                       UWord minKey, UWord minVal,
                       UWord maxKey, UWord maxVal,
                       UWord key,
                       Word(*kCmp)(UWord,UWord) );

// Clear the iterator stack.
void stackClear(WordFM* fm);

// Push onto the iterator stack.
void stackPush(WordFM* fm, AvlNode* n, Int i);

// Pop from the iterator stack.
Bool stackPop(WordFM* fm, AvlNode** n, Int* i);


AvlNode* avl_dopy ( AvlNode* nd,
                    UWord(*dopyK)(UWord),
                    UWord(*dopyV)(UWord) );

/* Initialise a WordFM. */
void initFM ( WordFM* fm,
                     Word    (*kCmp)(UWord,UWord) );

WordFM* VG_newFM ( Word(*kCmp)(UWord,UWord) );

void avl_free ( AvlNode* nd,
                       void(*kFin)(UWord),
                       void(*vFin)(UWord) );

/* Free up the FM.  If kFin is non-NULL, it is applied to keys
   before the FM is deleted; ditto with vFin for vals. */
void VG_deleteFM ( WordFM* fm, void(*kFin)(UWord), void(*vFin)(UWord) );

/* Add (k,v) to fm. */
Bool VG_addToFM ( WordFM* fm, UWord k, UWord v );

// Delete key from fm, returning associated key and val if found
Bool VG_delFromFM ( WordFM* fm,
                      /*OUT*/UWord* oldK, /*OUT*/UWord* oldV, UWord key );

// Look up in fm, assigning found key & val at spec'd addresses
Bool VG_lookupFM ( WordFM* fm,
                     /*OUT*/UWord* keyP, /*OUT*/UWord* valP, UWord key );

// See comment in pub_tool_wordfm.h for explanation
Bool VG_findBoundsFM ( WordFM* fm,
                        /*OUT*/UWord* kMinP, /*OUT*/UWord* vMinP,
                        /*OUT*/UWord* kMaxP, /*OUT*/UWord* vMaxP,
                        UWord minKey, UWord minVal,
                        UWord maxKey, UWord maxVal,
                        UWord key );

// See comment in pub_tool_wordfm.h for performance warning
UWord VG_sizeFM ( WordFM* fm );


// set up FM for iteration
void VG_initIterFM ( WordFM* fm );

// set up FM for iteration so that the first key subsequently produced
// by VG_(nextIterFM) is the smallest key in the map >= start_at.
// Naturally ">=" is defined by the comparison function supplied to
// VG_(newFM), as documented above.
void VG_initIterAtFM ( WordFM* fm, UWord start_at );
// get next key/val pair.  Will tl_assert if fm has been modified
// or looked up in since initIter{,At}FM was called.
Bool VG_nextIterFM ( WordFM* fm, /*OUT*/UWord* pKey, /*OUT*/UWord* pVal );

// clear the I'm iterating flag
void VG_doneIterFM ( WordFM* fm );


// admin: what's the 'common' allocation size (for tree nodes?)
SizeT VG_getNodeSizeFM( void );


/* Here's the comparison function.  Since the tree is required
to contain non-zero sized, non-overlapping blocks, it's good
enough to consider any overlap as a match. */
Word interval_tree_Cmp ( UWord k1, UWord k2 );

//------------------------------------------------------------------//
//---                         end WordFM                         ---//
//---                       Implementation                       ---//
//------------------------------------------------------------------//
#endif // __CPU_SIMPLE_WORDFM_HH__
