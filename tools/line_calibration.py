#!/usr/bin/env python3
"""Print per-sensor calibration defines from comma-separated raw ADC samples."""
import argparse
import statistics

def samples(text):
    try:
        values=[int(value.strip()) for value in text.split(',')]
    except ValueError as exc:
        raise argparse.ArgumentTypeError('Use raw integer ADC samples separated by commas') from exc
    if not values or any(value<0 or value>4095 for value in values):
        raise argparse.ArgumentTypeError('Each raw ADC sample must be0..4095')
    return round(statistics.median(values))

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for side in ('left','right'):
        for surface in ('white','black'):
            parser.add_argument(f'--{side}-{surface}',type=samples,required=True)
    args=parser.parse_args()
    for side in ('left','right'):
        white=getattr(args,side+'_white');black=getattr(args,side+'_black')
        if abs(black-white)<200:
            parser.error(f'{side}: white/black span is below200 counts; check AO, height and contrast')
    print('Replace these four values in app_program/subsystem3/line_config.h:')
    for side in ('left','right'):
        for surface in ('white','black'):
            print(f'#define S3_{surface.upper()}_{side.upper()} {getattr(args,side+"_"+surface)}')
    print('Either ADC polarity is supported. This tool does not write files or move motors.')

if __name__=='__main__':main()
