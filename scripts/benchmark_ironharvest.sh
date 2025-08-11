filename="$1" 
if [ -z "$1" ]; then
   echo "Please provide a filename"
   exit 1
fi 
{
read
while IFS=$'\t' read -r scen map temp; do
    cd ../build/
    echo "${map:5:50}"
    echo "${scen}"
    ./RayScan-jps --alg rjps --scen ${scen} --map ${map} --test n > ../../outputs/rjps__${map:5:50}.tsv
done 
}< "$filename"