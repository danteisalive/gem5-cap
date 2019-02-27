        .file	"fastbin_dup.c"
        .text
        .comm	Cap,16,16
        .section	.rodata
.LC0:
        .string	"In Range"
.LC1:
        .string	"Out of Range"
        .text
        .globl	main
        .type	main, @function
main:
.LFB5:
        .cfi_startproc
        pushq	%rbp
        .cfi_def_cfa_offset 16
        .cfi_offset 6, -16
        movq	%rsp, %rbp
        .cfi_def_cfa_register 6
        subq	$16, %rsp
        movl	$32, %edi
        call	malloc@PLT
        movq	%rax, -16(%rbp)
        movq	-16(%rbp), %rax
        movq	%rax, Cap(%rip)
        movq	-16(%rbp), %rax
        addq	$32, %rax
        movq	%rax, 8+Cap(%rip)
        movq	-16(%rbp), %rax
        addq	$129, %rax
        movq	%rax, -8(%rbp)
        movq	Cap(%rip), %rax
        cmpq	%rax, -8(%rbp)
        jb	.L2
        movq	8+Cap(%rip), %rax
        cmpq	%rax, -8(%rbp)
        jnb	.L2
        leaq	.LC0(%rip), %rdi
        call	puts@PLT
        jmp	.L3
.L2:
        leaq	.LC1(%rip), %rdi
        call	puts@PLT
.L3:
        movl	$0, %eax
        leave
        .cfi_def_cfa 7, 8
        ret
        .cfi_endproc
.LFE5:
        .size	main, .-main
        .ident	"GCC: (Ubuntu 7.3.0-27ubuntu1~18.04) 7.3.0"
        .section	.note.GNU-stack,"",@progbits
