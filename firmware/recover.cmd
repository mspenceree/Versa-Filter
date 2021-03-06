/* This is the command file to be used with the HEX conversion utility: dsphex.exe */
/* It reads the COFF output file "filt.out" and converts it to an ASCII hex file.  */
/*                                                                                 */
/* Required by Labview for serial Versa-Filter updating                            */
/*                                                                                 */
/* Invoke with: "dsphex recover.cmd"                                               */
/*                                                                                 */


recover.out                                 /* input file */
-o c:\work\sps\filter\labview\recover.asc   /* output file */
-map c:\work\sps\filter\labview\recover.txt /* report file */
-a                                          /* generate ASCII-Hex format output */
-romwidth 16                                /* put both bytes in same output file */
