import sys
sys.path.append('build')
import TimestampCorrector as TC

tc = TC.TimestampCorrector()
tc.correctTimestamp(100.22, 10.11)

