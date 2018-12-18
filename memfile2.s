// memfile2.s
// david_harris@hmc.edu and sarah.harris@unlv.edu 20 Jan 2014
// Test ARM processor
// ADD, SUB, AND, ORR, LDR, STR, B
// TST, LSL, CMN, ADC
// If successful, it should write the value 7 to address 100

MAIN	MOV R0, #3			; R0 = 3
		EOR R0, R0, #3		; Invert bits zero and one of R0
		SUB R0, R15, R15 	; R0 = 0				
		ADD R2, R0, #5      ; R2 = 5
		CMP R2, R0			; R2 > R0, flags Z == 0, N == 1
		ADD R3, R0, #12    	; R3 = 12         
		SUB R7, R3, #9    	; R7 = 3
		TST R7, R2			; set N and Z flags to 1 and 0
		ORR R4, R7, R2    	; R4 = 3 OR 5 = 7              	  		
       	AND R5, R3, R4    	; R5 = 12 AND 7 = 4            	
		ADD R5, R5, R4    	; R5 = 4 + 7 = 11              			
        SUBS R8, R5, R7    	; R8 <= 11 - 3 = 8, set Flags   	  		
        BEQ END        		; shouldn't be taken            	  		
        SUBS R8, R3, R4    	; R8 = 12 - 7  = 5            			
       	BGE AROUND       	; should be taken               
		ADD R5, R0, #0     	; should be skipped             	
AROUND   
		SUBS R8, R7, R2   	; R8 = 3 - 5 = -2, set Flags   	         	
        ADDLT R7, R5, #1  	; R7 = 11 + 1 = 12				          	
        SUB R7, R7, R2    	; R7 = 12 - 5 = 7			
    	STR R7, [R3, #84]  	; mem[12+84] = 7		     	
		LDR R2, [R0, #96]  	; R2 = mem[96] = 7
		LSL R3, R2, #3		; R2 = R2 << 3
		ADD R15, R15, R0	; PC <- PC + 8 (skips next)     	         
		ADD R2, R0, #14    	; shouldn't happen              	
		B END             	; always taken					
		ADD R2, R0, #13   	; shouldn't happen				
		ADD R2, R0, #10		; shouldn't happen
		LDRB R1, [R2, #1] 	; R1 = mem[R2,1]
END		STR R2, [R0, #100] 	; mem[100] = 7 	


// MAIN		 MOV R0, #3			; R0 = 3
//		1110 0011 1010 0000 0000 0000 0000 1100  E3A0000C 0x00
//			 EOR R0, R0, #3		; Invert bits zero and one of R0
// 		1110 0010 0010 0000 0000 0000 0000 0011  E2200003 0x04
//			 SUB R0, R15, R15 	; R0 = 0				
//		1110 000 0010 0 1111 0000 0000 0000 1111 E04F000F 0x08
//  		 ADD R2, R0, #5     ; R2 = 5             
//		1110 001 0100 0 0000 0010 0000 0000 0101 E2802005 0x0c
//			 CMP R2, R0			; R2 > R0, flags Z == 0, N == 1
//		1110 0001 0101 0010 0000 0000 0000 0000  E1520000 0x10
//  		 ADD R3, R0, #12    ; R3 = 12            
//		1110 001 0100 0 0000 0011 0000 0000 1100 E280300C 0x14
//  		 SUB R7, R3, #9    	; R7 = 3             
//		1110 001 0010 0 0011 0111 0000 0000 1001 E2437009 0x18
// 			 TST R7, R2			; set N and Z flags to 1 and 0
// 		1110 0001 0001 0111 0000 0000 0000 0010	 E1170002 0x1c
//  		 ORR R4, R7, R2    	; R4 = 3 OR 5 = 7              	
//      	1110 000 1100 0 0111 0100 0000 0000 0010 E1874002 0x20
//  		 AND R5, R3, R4    	; R5 = 12 AND 7 = 4            	
//     		1110 000 0000 0 0011 0101 0000 0000 0100 E0035004 0x24
//  		 ADD R5, R5, R4    	; R5 = 4 + 7 = 11              	
//      	1110 000 0100 0 0101 0101 0000 0000 0100 E0855004 0x28
//  		 SUBS R8, R5, R7    ; R8 <= 11 - 3 = 8, set Flags   	
//      	1110 000 0010 1 0101 1000 0000 0000 0111 E0558007 0x2c
//  		 BEQ END        	; shouldn't be taken            	
//      	0000 1010 0000  0000 0000 0000 0000 1100 0A00000C 0x30
//  		SUBS R8, R3, R4    ; R8 = 12 - 7  = 5             	
//     		 1110 000 0010 1 0011 1000 0000 0000 0100 E0538004 0x34
//  		 BGE AROUND       	; should be taken               	
//      	1010 1010 0000  0000 0000 0000 0000 0000 AA000000 0x38
//  		ADD R5, R0, #0     ; should be skipped             	
//      	1110 001 0100 0 0000 0101 0000 0000 0000 E2805000 0x3c
// AROUND   	SUBS R8, R7, R2   	; R8 = 3 - 5 = -2, set Flags   	
//      	1110 000 0010 1 0111 1000 0000 0000 0010 E0578002 0x40
//          	ADDLT R7, R5, #1  	; R7 = 11 + 1 = 12				
//      	1011 001 0100 0 0101 0111 0000 0000 0001 B2857001 0x44
//          	SUB R7, R7, R2    	; R7 = 12 - 5 = 7				
//      	1110 000 0010 0 0111 0111 0000 0000 0010 E0477002 0x48
//         	STR R7, [R3, #84]  	; mem[12+84] = 7		     	
//		1110 010 1100 0 0011 0111 0000 0101 0100 E5837054 0x4c
//          	LDR R2, [R0, #96]  	; R2 = mem[96] = 7				
//      	1110 010 1100 1 0000 0010 0000 0110 0000 E5902060 0x50
//		LSL R3, R2, #3		; R2 = R2 << 3
//		1110 0001 1010 0000 0010 0001 1000 0010	 E1A02182 0x54
//          	ADD R15, R15, R0	; PC <- PC + 8 (skips next)     	
//      	1110 000 0100 0 1111 1111 0000 0000 0000 E08FF000 0x58
//          	ADD R2, R0, #14    	; shouldn't happen              	
//      	1110 001 0100 0 0000 0010 0000 0000 0001 E280200E 0x5c
//          	B END             	; always taken					
//      	1110 1010 0000 0000 0000 0000 0000 0001  EA000001 0x60
//          	ADD R2, R0, #13   	; shouldn't happen				
//      	1110 001 0100 0 0000 0010 0000 0000 0001 E280200D 0x64
//          	ADD R2, R0, #10		; shouldn't happen				
//      	1110 001 0100 0 0000 0010 0000 0000 0001 E280200A 0x68
// 		LDRB R1, [R2, #1]	; R1 = mem[R2,1]
// 		1110 0101 1101 0010 0001 0000 0000 0001  E5D21001 0x6c
// END 		STR R2, [R0, #100] 	; mem[100] = 7                  	
//      	1110 010 1100 0 0000 0010 0000 0101 0100 E5802064 0x70
