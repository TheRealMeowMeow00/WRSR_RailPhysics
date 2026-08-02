// stubs.S - register-context bridges for railphysics' mid-function hooks.
//
// The loader's inline hook turns the site into a 14-byte absolute jump to the
// stub. The stub runs with the game's registers live, so it cannot be a C
// function: it saves every volatile register the helper may clobber, maps the
// game's values onto the Win64 argument registers, calls the helper, restores,
// replays the stolen instructions (with the helper's result in xmm0) and jumps
// to the continuation. The continuation addresses are runtime-computed (ASLR),
// so the stubs jump through .data slots filled in by TsmPluginInit.
//
// Win64 volatile: rax, rcx, rdx, r8-r11, xmm0-xmm5. xmm6-xmm15 and the
// non-volatile integer registers survive the C helper by the ABI - which is
// what keeps the game's xmm6/xmm9/xmm10/xmm14/xmm15, rsi and rbp intact.

    .intel_syntax noprefix
    .text

// ---------------------------------------------------------------- brake
// Site 0x1406A8443, 16 stolen bytes:
//     subss  xmm6, xmm0                    ; speed -= decrement
//     movss  [rsi+0xd24], xmm6
//     comiss xmm9, xmm6                    ; flags feed the JBE at the continuation
// In:  xmm0 = vanilla decrement this frame, xmm6 = speed, xmm15 = divisor,
//      rsi = vehicle. Helper returns the replacement decrement in xmm0.

    .globl rp_brake_stub
    .def   rp_brake_stub; .scl 2; .type 32; .endef
rp_brake_stub:
    push    rbp
    mov     rbp, rsp
    and     rsp, -16
    sub     rsp, 0xA0
    mov     [rsp+0x00], rax
    mov     [rsp+0x08], rcx
    mov     [rsp+0x10], rdx
    mov     [rsp+0x18], r8
    mov     [rsp+0x20], r9
    mov     [rsp+0x28], r10
    mov     [rsp+0x30], r11
    movaps  [rsp+0x40], xmm1
    movaps  [rsp+0x50], xmm2
    movaps  [rsp+0x60], xmm3
    movaps  [rsp+0x70], xmm4
    movaps  [rsp+0x80], xmm5

    mov     rcx, rsi                        // vehicle
    movaps  xmm1, xmm6                      // speed
    movaps  xmm2, xmm0                      // vanilla increment
    movaps  xmm3, xmm15                     // divisor (possibly emergency-scaled)
    movaps  xmm4, xmm9                      // effective limit (post curve-clamp)
    call    rp_brake_helper

    mov     rax, [rsp+0x00]
    mov     rcx, [rsp+0x08]
    mov     rdx, [rsp+0x10]
    mov     r8,  [rsp+0x18]
    mov     r9,  [rsp+0x20]
    mov     r10, [rsp+0x28]
    mov     r11, [rsp+0x30]
    movaps  xmm1, [rsp+0x40]
    movaps  xmm2, [rsp+0x50]
    movaps  xmm3, [rsp+0x60]
    movaps  xmm4, [rsp+0x70]
    movaps  xmm5, [rsp+0x80]
    mov     rsp, rbp
    pop     rbp

    subss   xmm6, xmm0                      // stolen block, with our decrement
    movss   dword ptr [rsi+0xd24], xmm6
    comiss  xmm9, xmm6
    jmp     qword ptr [rip+rp_brake_back]

// ---------------------------------------------------------------- slope
// Site 0x1406A8566, 21 stolen bytes:
//     mulss  xmm0, xmm10                   ; increment *= s (vanilla drag)
//     addss  xmm0, [rsi+0xd24]
//     movss  [rsi+0xd24], xmm0
// In:  xmm0 = vanilla per-frame increment, xmm10 = s = slopeAvg*-0.5,
//      xmm15 = divisor, rsi = vehicle. Helper returns a signed gravity delta;
//      the continuation (0x1406A857B) clamps the result to >= 0 as in vanilla.

    .globl rp_slope_stub
    .def   rp_slope_stub; .scl 2; .type 32; .endef
rp_slope_stub:
    push    rbp
    mov     rbp, rsp
    and     rsp, -16
    sub     rsp, 0xA0
    mov     [rsp+0x00], rax
    mov     [rsp+0x08], rcx
    mov     [rsp+0x10], rdx
    mov     [rsp+0x18], r8
    mov     [rsp+0x20], r9
    mov     [rsp+0x28], r10
    mov     [rsp+0x30], r11
    movaps  [rsp+0x40], xmm1
    movaps  [rsp+0x50], xmm2
    movaps  [rsp+0x60], xmm3
    movaps  [rsp+0x70], xmm4
    movaps  [rsp+0x80], xmm5

    mov     rcx, rsi                        // vehicle
    movaps  xmm1, xmm0                      // vanilla increment
    movaps  xmm2, xmm10                     // s
    movaps  xmm3, xmm15                     // divisor
    call    rp_slope_helper

    mov     rax, [rsp+0x00]
    mov     rcx, [rsp+0x08]
    mov     rdx, [rsp+0x10]
    mov     r8,  [rsp+0x18]
    mov     r9,  [rsp+0x20]
    mov     r10, [rsp+0x28]
    mov     r11, [rsp+0x30]
    movaps  xmm1, [rsp+0x40]
    movaps  xmm2, [rsp+0x50]
    movaps  xmm3, [rsp+0x60]
    movaps  xmm4, [rsp+0x70]
    movaps  xmm5, [rsp+0x80]
    mov     rsp, rbp
    pop     rbp

    addss   xmm0, dword ptr [rsi+0xd24]     // apply our signed delta
    movss   dword ptr [rsi+0xd24], xmm0
    jmp     qword ptr [rip+rp_slope_back]

// ---------------------------------------------------------------- curve limit
// Site 0x1406A8326, 17 stolen bytes:
//     cmp   byte ptr [rsi+0xd41], r13b   ; roll-out flag
//     jz    0x1406A8380                  ; rel8 - replayed as an absolute jump
//     movss xmm6, [rsi+0xd24]
// In:  rsi = train, xmm9 = final effective limit (km/h). The helper returns
// min(limit, curveLimit); we move it into xmm9 (non-volatile, so it survives
// the call), then replay the stolen block, routing the jz through a second
// address slot.

    .globl rp_curve_stub
    .def   rp_curve_stub; .scl 2; .type 32; .endef
rp_curve_stub:
    push    rbp
    mov     rbp, rsp
    and     rsp, -16
    sub     rsp, 0xA0
    mov     [rsp+0x00], rax
    mov     [rsp+0x08], rcx
    mov     [rsp+0x10], rdx
    mov     [rsp+0x18], r8
    mov     [rsp+0x20], r9
    mov     [rsp+0x28], r10
    mov     [rsp+0x30], r11
    movaps  [rsp+0x40], xmm1
    movaps  [rsp+0x50], xmm2
    movaps  [rsp+0x60], xmm3
    movaps  [rsp+0x70], xmm4
    movaps  [rsp+0x80], xmm5

    mov     rcx, rsi                        // vehicle
    movaps  xmm1, xmm9                      // vanilla limit
    call    rp_curve_helper

    movaps  xmm9, xmm0                      // clamped limit

    mov     rax, [rsp+0x00]
    mov     rcx, [rsp+0x08]
    mov     rdx, [rsp+0x10]
    mov     r8,  [rsp+0x18]
    mov     r9,  [rsp+0x20]
    mov     r10, [rsp+0x28]
    mov     r11, [rsp+0x30]
    movaps  xmm1, [rsp+0x40]
    movaps  xmm2, [rsp+0x50]
    movaps  xmm3, [rsp+0x60]
    movaps  xmm4, [rsp+0x70]
    movaps  xmm5, [rsp+0x80]
    mov     rsp, rbp
    pop     rbp

    cmp     byte ptr [rsi+0xd41], r13b      // stolen block
    jz      .Lcurve_branch
    movss   xmm6, dword ptr [rsi+0xd24]
    jmp     qword ptr [rip+rp_curve_back]
.Lcurve_branch:
    jmp     qword ptr [rip+rp_curve_branch]

// ---------------------------------------------------------------- grid boost
// Site 0x1401BDF1C, 16 stolen bytes (plant outflow ceiling computation):
//     mov   rax, [rbp+0x970]
//     mulss xmm0, [rdi+rbx+0x8c]       ; SIB addressing - keep rbx intact!
// No register context needed: replay the stolen block verbatim, then scale
// xmm0 by the configured multiplier and jump back.

    .globl rp_grid_stub
    .def   rp_grid_stub; .scl 2; .type 32; .endef
rp_grid_stub:
    mov     rax, qword ptr [rbp+0x970]
    mulss   xmm0, dword ptr [rdi+rbx+0x8c]
    mulss   xmm0, dword ptr [rip+rp_grid_k]
    jmp     qword ptr [rip+rp_grid_back]

// ---------------------------------------------------------------- jump tables

    .data
    .align 8
    .globl rp_brake_back
rp_brake_back:
    .quad 0
    .globl rp_slope_back
rp_slope_back:
    .quad 0
    .globl rp_curve_back
rp_curve_back:
    .quad 0
    .globl rp_curve_branch
rp_curve_branch:
    .quad 0
    .globl rp_grid_back
rp_grid_back:
    .quad 0
    .globl rp_grid_k
rp_grid_k:
    .float 1.0
