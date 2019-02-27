#include <stdio.h>
#include <stdlib.h>

#include <iostream>

struct Capability {
        void * begin;
        void * end;
        //bool free;

} Cap;



int main()
{
        //fprintf(stderr, "This file demonstrates a simple double-free attack with fastbins.\n");
        //u_int64_t Cap = 129;
        //u_int64_t Jmp = 0x3E;
        //fprintf(stderr, "Allocating 3 buffers.\n");
        void *a = malloc(32);
        Cap.begin = a;
        Cap.end = a + 32;
        //Cap.free = false;
        u_int64_t offset;
        std::cin >> offset;
    void *b = a + offset;

        if (b >= Cap.begin && b < Cap.end){

                printf("In Range\n");
        }
        else{

                printf("Out of Range\n");
        }


        //void *b = a + 32;
        //void *b = a;
        //printf("a Address: %p\n", a );

        //for (int i = 0; i < 38; i++){
        //	a += 1;
                //if (b >= a && b <= (a + 38) )
        //	printf( "%d\n", (int)a);
                //b = b+i;
        //}

        //printf("Capability: %#lx\n", Cap );

        //for (int i = 0; i < 4; i++)
        //	a[i] += 1;

        //for (int i = 0; i < 4; i++)
        //	printf("%d\n", a[i]);;
        //int *b = malloc(8);
        //int *c = malloc(8);


        //fprintf(stderr, "1st malloc(8): %p\n", a);
        //fprintf(stderr, "2nd malloc(8): %p\n", b);
        //fprintf(stderr, "3rd malloc(8): %p\n", c);

        //fprintf(stderr, "Freeing the first one...\n");
        //free(a);

        //fprintf(stderr, "If we free %p again, things will crash because %p is at the top of the free list.\n", a, a);
        // free(a);

        //fprintf(stderr, "So, instead, we'll free %p.\n", b);
        //free(b);

        //fprintf(stderr, "Now, we can free %p again, since it's not the head of the free list.\n", a);
        //free(a);

        //fprintf(stderr, "Now the free list has [ %p, %p, %p ]. If we malloc 3 times, we'll get %p twice!\n", a, b, a, a);
        //int *d = malloc(8);
        //int *e = malloc(8);
        //int *f = malloc(8);
        //fprintf(stderr, "1st malloc(8): %p\n", d);
        //fprintf(stderr, "2nd malloc(8): %p\n", e);
        //fprintf(stderr, "3rd malloc(8): %p\n", f);
        return 0;
}
