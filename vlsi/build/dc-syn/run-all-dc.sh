for i in `seq 1 4`;
do
  for j in `seq 1 4`;
  do
    for k in `seq 1 1`;
    do
      export clock_period="((0.5/$j))"
      ./run-dc.sh $i $j $k
      cp -R current-dc/reports by-stage-target-freq-reports/no-retiming-reports/reports$i$j$k
    done
  done
done

for i in `seq 1 4`;
do
  for j in `seq 1 $i`;
  do
    for k in `seq 0 0`;
    do
      export clock_period="((0.5/$j))"
      ./run-dc.sh $i $j $k
      cp -R current-dc/reports by-stage-target-freq-reports/no-retiming-reports/reports$i$j$k
    done
  done
done
