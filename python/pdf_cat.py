#!/usr/bin/env python3
# import contextlib
import os
import sys
try:
    from PyPDF2 import PdfReader, PdfWriter
except ImportError:
    from pyPdf import PdfReader, PdfWriter

def pdf_cat(input_files, output_stream):
    input_streams = []
    try:
        # First open all the files, then produce the output file, and
        # finally close the input files. This is necessary because
        # the data isn't read from the input files until the write
        # operation. Thanks to
        # https://stackoverflow.com/questions/6773631/problem-with-closing-python-pypdf-writing-getting-a-valueerror-i-o-operation/6773733#6773733
        for input_file in input_files:
            input_streams.append(open(input_file, 'rb'))
        writer = PdfWriter()
        for reader in map(PdfReader, input_streams):
            for n in range(len(reader.pages)):
                writer.add_page(reader.pages[n])
        writer.write(output_stream)
    finally:
        for f in input_streams:
            f.close()

if __name__ == '__main__':
    if sys.platform == "win32":
        import os, msvcrt
        msvcrt.setmode(sys.stdout.fileno(), os.O_BINARY)
    if len(sys.argv) < 2:
        print('Usage: {} a.pdf b.pdf [...]'.format(sys.argv[0]))
        sys.exit(1)
    output_file = os.path.join(os.path.dirname(sys.argv[1]), "combined.pdf")
    with open(output_file, "wb") as output_stream:
        pdf_cat(sys.argv[1:], output_stream)

    print("The combined PDF is saved to {}.".format(output_file))


