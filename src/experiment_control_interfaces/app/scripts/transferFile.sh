# Bash script

FILE=$1
NSOLUTION=$2
DIROUTPUT=$3
NINPUTS=$4
NOUTPUTS=$5
NHIDDEN=$6


rm -f ${DIROUTPUT}/solution_${NSOLUTION}.csv


echo ${NINPUTS} >> ${DIROUTPUT}/solution_${NSOLUTION}.csv
echo ${NOUTPUTS} >> ${DIROUTPUT}/solution_${NSOLUTION}.csv
echo ${NHIDDEN} >> ${DIROUTPUT}/solution_${NSOLUTION}.csv


sed -n ${NSOLUTION}'p' ${FILE} >> ${DIROUTPUT}/solution_${NSOLUTION}.csv