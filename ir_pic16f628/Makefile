default: dec-043.hex

%.o : %.asm
	gpasm -c $<

enc-042.hex: enc-042.o mtxv4.o
	gplink -s 16f630.lkr -o $@ $^

enc-043.hex: enc-043.o irmtxv4.o
	gplink -s 16f684.lkr -o $@ $^

dec-043.hex: dec-043.o mrxv4.o
	gplink -s 16f628.lkr -o $@ $^

dec-044.hex: dec-044.o mrxv4.o
	gplink -s 16f628.lkr -o $@ $^

clean:
	 rm -f *.o *.hex *.0 *.lst
