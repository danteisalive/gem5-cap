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

#include "cpu/simple/WordFM.hh"

/* Swing to the left.  Warning: no balance maintainance. */
void avl_swl ( AvlNode** root )
{
   AvlNode* a  = *root;
   AvlNode* b  = a->child[1];
   *root       = b;
   a->child[1] = b->child[0];
   b->child[0] = a;
}

/* Swing to the right.  Warning: no balance maintainance. */
void avl_swr ( AvlNode** root )
{
   AvlNode* a  = *root;
   AvlNode* b  = a->child[0];
   *root       = b;
   a->child[0] = b->child[1];
   b->child[1] = a;
}

/* Balance maintainance after especially nasty swings. */
void avl_nasty ( AvlNode* root )
{
   switch (root->balance) {
      case -1:
         root->child[0]->balance = 0;
         root->child[1]->balance = 1;
         break;
      case 1:
         root->child[0]->balance = -1;
         root->child[1]->balance = 0;
         break;
      case 0:
         root->child[0]->balance = 0;
         root->child[1]->balance = 0;
         break;
      default:
         assert(0);
   }
   root->balance=0;
}

/* Find size of a non-NULL tree. */
UWord size_avl_nonNull ( AvlNode* nd )
{
   return 1 + (nd->child[0] ? size_avl_nonNull(nd->child[0]) : 0)
            + (nd->child[1] ? size_avl_nonNull(nd->child[1]) : 0);
}

/* Unsignedly compare w1 and w2.  If w1 < w2, produce a negative
   number; if w1 > w2 produce a positive number, and if w1 == w2
   produce zero. */
Word cmp_unsigned_Words ( UWord w1, UWord w2 ) {
   if (w1 < w2) return -1;
   if (w1 > w2) return 1;
   return 0;
}

/* Insert element a into the AVL tree t.  Returns True if the depth of
   the tree has grown.  If element with that key is already present,
   just copy a->val to existing node, first returning old ->val field
   of existing node in *oldV, so that the caller can finalize it
   however it wants.
*/

Bool avl_insert_wrk ( AvlNode**         rootp,
                      /*OUT*/MaybeWord* oldV,
                      AvlNode*          a,
                      Word              (*kCmp)(UWord,UWord) )
{
   Word cmpres;

   /* initialize */
   a->child[0] = 0;
   a->child[1] = 0;
   a->balance  = 0;
   oldV->b     = False;

   /* insert into an empty tree? */
   if (!(*rootp)) {
      (*rootp) = a;
      return True;
   }

   cmpres = kCmp ? /*boxed*/   kCmp( (*rootp)->key, a->key )
                 : /*unboxed*/ cmp_unsigned_Words( (UWord)(*rootp)->key,
                                                   (UWord)a->key );

   if (cmpres > 0) {
      /* insert into the left subtree */
      if ((*rootp)->child[0]) {
         AvlNode* left_subtree = (*rootp)->child[0];
         if (avl_insert_wrk(&left_subtree, oldV, a, kCmp)) {
            switch ((*rootp)->balance--) {
               case  1: return False;
               case  0: return True;
               case -1: break;
               default: assert(0);
            }
            if ((*rootp)->child[0]->balance < 0) {
               avl_swr( rootp );
               (*rootp)->balance = 0;
               (*rootp)->child[1]->balance = 0;
            } else {
               avl_swl( &((*rootp)->child[0]) );
               avl_swr( rootp );
               avl_nasty( *rootp );
            }
         } else {
            (*rootp)->child[0] = left_subtree;
         }
         return False;
      } else {
         (*rootp)->child[0] = a;
         if ((*rootp)->balance--)
            return False;
         return True;
      }
      assert(0);/*NOTREACHED*/
   }
   else
   if (cmpres < 0) {
      /* insert into the right subtree */
      if ((*rootp)->child[1]) {
         AvlNode* right_subtree = (*rootp)->child[1];
         if (avl_insert_wrk(&right_subtree, oldV, a, kCmp)) {
            switch((*rootp)->balance++){
               case -1: return False;
               case  0: return True;
               case  1: break;
               default: assert(0);
            }
            if ((*rootp)->child[1]->balance > 0) {
               avl_swl( rootp );
               (*rootp)->balance = 0;
               (*rootp)->child[0]->balance = 0;
            } else {
               avl_swr( &((*rootp)->child[1]) );
               avl_swl( rootp );
               avl_nasty( *rootp );
            }
         } else {
            (*rootp)->child[1] = right_subtree;
         }
         return False;
      } else {
         (*rootp)->child[1] = a;
         if ((*rootp)->balance++)
            return False;
         return True;
      }
      assert(0);/*NOTREACHED*/
   }
   else {
      /* cmpres == 0, a duplicate - replace the val, but don't
         incorporate the node in the tree */
      oldV->b = True;
      oldV->w = (*rootp)->val;
      (*rootp)->val = a->val;
      return False;
   }
}


/* Remove an element a from the AVL tree t.  a must be part of
   the tree.  Returns True if the depth of the tree has shrunk.
*/

Bool avl_remove_wrk ( AvlNode** rootp,
                      AvlNode*  a,
                      Word(*kCmp)(UWord,UWord) )
{
   Bool ch;
   Word cmpres;
   cmpres = kCmp ? /*boxed*/   kCmp( (*rootp)->key, a->key )
                 : /*unboxed*/ cmp_unsigned_Words( (UWord)(*rootp)->key,
                                                   (UWord)a->key );

   if (cmpres > 0){
      /* remove from the left subtree */
      AvlNode* left_subtree = (*rootp)->child[0];
      assert(left_subtree);
      ch = avl_remove_wrk(&left_subtree, a, kCmp);
      (*rootp)->child[0]=left_subtree;
      if (ch) {
         switch ((*rootp)->balance++) {
            case -1: return True;
            case  0: return False;
            case  1: break;
            default: assert(0);
         }
         switch ((*rootp)->child[1]->balance) {
            case 0:
               avl_swl( rootp );
               (*rootp)->balance = -1;
               (*rootp)->child[0]->balance = 1;
               return False;
            case 1:
               avl_swl( rootp );
               (*rootp)->balance = 0;
               (*rootp)->child[0]->balance = 0;
               return True;
            case -1:
               break;
            default:
               assert(0);
         }
         avl_swr( &((*rootp)->child[1]) );
         avl_swl( rootp );
         avl_nasty( *rootp );
         return True;
      }
   }
   else
   if (cmpres < 0) {
      /* remove from the right subtree */
      AvlNode* right_subtree = (*rootp)->child[1];
      assert(right_subtree);
      ch = avl_remove_wrk(&right_subtree, a, kCmp);
      (*rootp)->child[1] = right_subtree;
      if (ch) {
         switch ((*rootp)->balance--) {
            case  1: return True;
            case  0: return False;
            case -1: break;
            default: assert(0);
         }
         switch ((*rootp)->child[0]->balance) {
            case 0:
               avl_swr( rootp );
               (*rootp)->balance = 1;
               (*rootp)->child[1]->balance = -1;
               return False;
            case -1:
               avl_swr( rootp );
               (*rootp)->balance = 0;
               (*rootp)->child[1]->balance = 0;
               return True;
            case 1:
               break;
            default:
               assert(0);
         }
         avl_swl( &((*rootp)->child[0]) );
         avl_swr( rootp );
         avl_nasty( *rootp );
         return True;
      }
   }
   else {
      assert(cmpres == 0);
      assert((*rootp)==a);
      return avl_removeroot_wrk(rootp, kCmp);
   }
   return 0;
}


/* Remove the root of the AVL tree *rootp.
 * Warning: dumps core if *rootp is empty
 */

Bool avl_removeroot_wrk( AvlNode** rootp, Word(*kCmp)(UWord,UWord) )
{
   Bool     ch;
   AvlNode* a;
   if (!(*rootp)->child[0]) {
      if (!(*rootp)->child[1]) {
         (*rootp) = 0;
         return True;
      }
      (*rootp) = (*rootp)->child[1];
      return True;
   }
   if (!(*rootp)->child[1]) {
      (*rootp) = (*rootp)->child[0];
      return True;
   }
   if ((*rootp)->balance < 0) {
      /* remove from the left subtree */
      a = (*rootp)->child[0];
      while (a->child[1]) a = a->child[1];
   } else {
      /* remove from the right subtree */
      a = (*rootp)->child[1];
      while (a->child[0]) a = a->child[0];
   }
   ch = avl_remove_wrk(rootp, a, kCmp);
   a->child[0] = (*rootp)->child[0];
   a->child[1] = (*rootp)->child[1];
   a->balance  = (*rootp)->balance;
   (*rootp)    = a;
   if (a->balance == 0) return ch;
   return False;
}



AvlNode* avl_find_node ( AvlNode* t, Word k, Word(*kCmp)(UWord,UWord) )
{
   if (kCmp) {
      /* Boxed comparisons */
      Word cmpresS;
      while (True) {
         if (t == NULL) return NULL;
         cmpresS = kCmp(t->key, k);
         if (cmpresS > 0) t = t->child[0]; else
         if (cmpresS < 0) t = t->child[1]; else
         return t;
      }
   } else {
      /* Unboxed comparisons */
      Word  cmpresS; /* signed */
      UWord cmpresU; /* unsigned */
      while (True) {
         if (t == NULL) return NULL; /* unlikely ==> predictable */
         cmpresS = cmp_unsigned_Words( (UWord)t->key, (UWord)k );
         if (cmpresS == 0) return t; /* unlikely ==> predictable */
         cmpresU = (UWord)cmpresS;
         cmpresU >>=/*unsigned*/ (8 * sizeof(cmpresU) - 1);
         t = t->child[cmpresU];
      }
   }
}


Bool avl_find_bounds ( AvlNode* t,
                       /*OUT*/UWord* kMinP, /*OUT*/UWord* vMinP,
                       /*OUT*/UWord* kMaxP, /*OUT*/UWord* vMaxP,
                       UWord minKey, UWord minVal,
                       UWord maxKey, UWord maxVal,
                       UWord key,
                       Word(*kCmp)(UWord,UWord) )
{
   UWord kLowerBound = minKey;
   UWord vLowerBound = minVal;
   UWord kUpperBound = maxKey;
   UWord vUpperBound = maxVal;
   while (t) {
      Word cmpresS = kCmp ? kCmp(t->key, key)
                          : cmp_unsigned_Words(t->key, key);
      if (cmpresS < 0) {
         kLowerBound = t->key;
         vLowerBound = t->val;
         t = t->child[1];
         continue;
      }
      if (cmpresS > 0) {
         kUpperBound = t->key;
         vUpperBound = t->val;
         t = t->child[0];
         continue;
      }
      /* We should never get here.  If we do, it means the given key
         is actually present in the tree, which means the original
         call was invalid -- an error on the caller's part, and we
         cannot give any meaningful values for the bounds.  (Well,
         maybe we could, but we're not gonna.  Ner!) */
      return False;
   }
   if (kMinP) *kMinP = kLowerBound;
   if (vMinP) *vMinP = vLowerBound;
   if (kMaxP) *kMaxP = kUpperBound;
   if (vMaxP) *vMaxP = vUpperBound;
   return True;
}

// Clear the iterator stack.
void stackClear(WordFM* fm)
{
   Int i;
   assert(fm);
   for (i = 0; i < WFM_STKMAX; i++) {
      fm->nodeStack[i] = NULL;
      fm->numStack[i]  = 0;
   }
   fm->stackTop = 0;
}

// Push onto the iterator stack.
void stackPush(WordFM* fm, AvlNode* n, Int i)
{
   assert(fm->stackTop < WFM_STKMAX);
   assert(1 <= i && i <= 3);
   fm->nodeStack[fm->stackTop] = n;
   fm-> numStack[fm->stackTop] = i;
   fm->stackTop++;
}

// Pop from the iterator stack.
Bool stackPop(WordFM* fm, AvlNode** n, Int* i)
{
   assert(fm->stackTop <= WFM_STKMAX);

   if (fm->stackTop > 0) {
      fm->stackTop--;
      *n = fm->nodeStack[fm->stackTop];
      *i = fm-> numStack[fm->stackTop];
      assert(1 <= *i && *i <= 3);
      fm->nodeStack[fm->stackTop] = NULL;
      fm-> numStack[fm->stackTop] = 0;
      return True;
   } else {
      return False;
   }
}


AvlNode* avl_dopy ( AvlNode* nd,
                    UWord(*dopyK)(UWord),
                    UWord(*dopyV)(UWord) )
{
   AvlNode* nyu;
   if (! nd)
      return NULL;
   nyu = static_cast<AvlNode *>(malloc(sizeof(AvlNode)));
   assert(nyu);

   nyu->child[0] = nd->child[0];
   nyu->child[1] = nd->child[1];
   nyu->balance = nd->balance;

   /* Copy key */
   if (dopyK) {
      nyu->key = dopyK( nd->key );
      if (nd->key != 0 && nyu->key == 0)
         return NULL; /* oom in key dcopy */
   } else {
      /* copying assumedly unboxed keys */
      nyu->key = nd->key;
   }

   /* Copy val */
   if (dopyV) {
      nyu->val = dopyV( nd->val );
      if (nd->val != 0 && nyu->val == 0)
         return NULL; /* oom in val dcopy */
   } else {
      /* copying assumedly unboxed vals */
      nyu->val = nd->val;
   }

   /* Copy subtrees */
   if (nyu->child[0]) {
      nyu->child[0] = avl_dopy( nyu->child[0], dopyK, dopyV );
      if (! nyu->child[0])
         return NULL;
   }
   if (nyu->child[1]) {
      nyu->child[1] = avl_dopy( nyu->child[1], dopyK, dopyV);
      if (! nyu->child[1])
         return NULL;
   }

   return nyu;
}

/* Initialise a WordFM. */
void initFM ( WordFM* fm,
                     Word    (*kCmp)(UWord,UWord) )
{
   fm->root         = 0;
   fm->kCmp         = kCmp;
   fm->stackTop     = 0;
}

WordFM* VG_newFM ( Word(*kCmp)(UWord,UWord) )
{
   WordFM* fm = static_cast<WordFM*>(malloc(sizeof(WordFM)));
   assert(fm);
   initFM(fm, kCmp);
   return fm;
}

void avl_free ( AvlNode* nd,
                       void(*kFin)(UWord),
                       void(*vFin)(UWord) )
{
   if (!nd)
      return;
   if (nd->child[0])
      avl_free(nd->child[0], kFin, vFin);
   if (nd->child[1])
      avl_free(nd->child[1], kFin, vFin);
   if (kFin)
      kFin( nd->key );
   if (vFin)
      vFin( nd->val );
   memset(nd, 0, sizeof(AvlNode));
   free(nd);
}

/* Free up the FM.  If kFin is non-NULL, it is applied to keys
   before the FM is deleted; ditto with vFin for vals. */
void VG_deleteFM ( WordFM* fm, void(*kFin)(UWord), void(*vFin)(UWord) )
{
   avl_free( fm->root, kFin, vFin );
   memset(fm, 0, sizeof(WordFM) );
   free(fm);
}

/* Add (k,v) to fm. */
Bool VG_addToFM ( WordFM* fm, UWord k, UWord v )
{
   MaybeWord oldV;
   AvlNode* node;
   node = static_cast<AvlNode*>(malloc(sizeof(AvlNode)));
   node->key = k;
   node->val = v;
   oldV.b = False;
   oldV.w = 0;
   avl_insert_wrk( &fm->root, &oldV, node, fm->kCmp );
   //if (oldV.b && fm->vFin)
   //   fm->vFin( oldV.w );
   if (oldV.b)
      free(node);
   return oldV.b;
}

// Delete key from fm, returning associated key and val if found
Bool VG_delFromFM ( WordFM* fm,
                      /*OUT*/UWord* oldK, /*OUT*/UWord* oldV, UWord key )
{
   AvlNode* node = avl_find_node( fm->root, key, fm->kCmp );
   if (node) {
      avl_remove_wrk( &fm->root, node, fm->kCmp );
      if (oldK)
         *oldK = node->key;
      if (oldV)
         *oldV = node->val;
      free(node);
      return True;
   } else {
      return False;
   }
}

// Look up in fm, assigning found key & val at spec'd addresses
Bool VG_lookupFM ( WordFM* fm,
                     /*OUT*/UWord* keyP, /*OUT*/UWord* valP, UWord key )
{
   AvlNode* node = avl_find_node( fm->root, key, fm->kCmp );
   if (node) {
      if (keyP)
         *keyP = node->key;
      if (valP)
         *valP = node->val;
      return True;
   } else {
      return False;
   }
}

// See comment in pub_tool_wordfm.h for explanation
Bool VG_findBoundsFM ( WordFM* fm,
                        /*OUT*/UWord* kMinP, /*OUT*/UWord* vMinP,
                        /*OUT*/UWord* kMaxP, /*OUT*/UWord* vMaxP,
                        UWord minKey, UWord minVal,
                        UWord maxKey, UWord maxVal,
                        UWord key )
{
   /* really we should assert that minKey <= key <= maxKey,
      where <= is as defined by fm->kCmp. */
   return avl_find_bounds( fm->root, kMinP, vMinP,
                                     kMaxP, vMaxP,
                                     minKey, minVal,
                                     maxKey, maxVal,
                                     key, fm->kCmp );
}

// See comment in pub_tool_wordfm.h for performance warning
UWord VG_sizeFM ( WordFM* fm )
{
   // Hmm, this is a bad way to do this
   return fm->root ? size_avl_nonNull( fm->root ) : 0;
}


// set up FM for iteration
void VG_initIterFM ( WordFM* fm )
{
   assert(fm);
   stackClear(fm);
   if (fm->root)
      stackPush(fm, fm->root, 1);
}

// set up FM for iteration so that the first key subsequently produced
// by VG_(nextIterFM) is the smallest key in the map >= start_at.
// Naturally ">=" is defined by the comparison function supplied to
// VG_(newFM), as documented above.
void VG_initIterAtFM ( WordFM* fm, UWord start_at )
{
   Int     i;
   AvlNode *n, *t;
   Word    cmpresS; /* signed */
   UWord   cmpresU; /* unsigned */

   assert(fm);
   stackClear(fm);

   if (!fm->root)
      return;

   n = NULL;
   // We need to do regular search and fill in the stack.
   t = fm->root;

   while (True) {
      if (t == NULL) return;

      cmpresS
         = fm->kCmp ? /*boxed*/   fm->kCmp( t->key, start_at )
                    : /*unboxed*/ cmp_unsigned_Words( t->key, start_at );

      if (cmpresS == 0) {
         // We found the exact key -- we are done.
         // The iteration should start with this node.
         stackPush(fm, t, 2);
         // The stack now looks like {2, 2, ... ,2, 2}
         return;
      }
      cmpresU = (UWord)cmpresS;
      cmpresU >>=/*unsigned*/ (8 * sizeof(cmpresU) - 1);
      if (!cmpresU) {
         // Push this node only if we go to the left child.
         stackPush(fm, t, 2);
      }
      t = t->child[cmpresU];
   }
   if (stackPop(fm, &n, &i)) {
      // If we've pushed something to stack and did not find the exact key,
      // we must fix the top element of stack.
      assert(i == 2);
      stackPush(fm, n, 3);
      // the stack looks like {2, 2, ..., 2, 3}
   }
}

// get next key/val pair.  Will tl_assert if fm has been modified
// or looked up in since initIter{,At}FM was called.
Bool VG_nextIterFM ( WordFM* fm, /*OUT*/UWord* pKey, /*OUT*/UWord* pVal )
{
   Int i = 0;
   AvlNode* n = NULL;

   assert(fm);

   // This in-order traversal requires each node to be pushed and popped
   // three times.  These could be avoided by updating nodes in-situ on the
   // top of the stack, but the push/pop cost is so small that it's worth
   // keeping this loop in this simpler form.
   while (stackPop(fm, &n, &i)) {
      switch (i) {
      case 1: case_1:
         stackPush(fm, n, 2);
         /* if (n->child[0])  stackPush(fm, n->child[0], 1); */
         if (n->child[0]) { n = n->child[0]; goto case_1; }
         break;
      case 2:
         stackPush(fm, n, 3);
         if (pKey) *pKey = n->key;
         if (pVal) *pVal = n->val;
         return True;
      case 3:
         /* if (n->child[1]) stackPush(fm, n->child[1], 1); */
         if (n->child[1]) { n = n->child[1]; goto case_1; }
         break;
      default:
         assert(0);
      }
   }

   // Stack empty, iterator is exhausted, return NULL
   return False;
}

// clear the I'm iterating flag
void VG_doneIterFM ( WordFM* fm )
{
}


// admin: what's the 'common' allocation size (for tree nodes?)
SizeT VG_getNodeSizeFM( void )
{
   return sizeof(AvlNode);
}

Word interval_tree_Cmp ( UWord k1, UWord k2 )
{
   Block* b1 = (Block*)k1;
   Block* b2 = (Block*)k2;
   assert(b1->req_szB > 0);
   assert(b2->req_szB > 0);
   if (b1->payload + b1->req_szB <= b2->payload) return -1;
   if (b2->payload + b2->req_szB <= b1->payload) return  1;
   return 0;
}
//------------------------------------------------------------------//
//---                         end WordFM                         ---//
//---                       Implementation                       ---//
//------------------------------------------------------------------//
