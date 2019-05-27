


package main

import "fmt"
import "io/ioutil"
import "os"
import "bufio"
import "strings"
import "strconv"
import "math"

type gps struct{
    lat float64
    lon float64
    index int
    timestamp int
    tsFloat float64
    speedInv float64
    total_distance float64

}

func distance(g1 gps, g2 gps) float64 {
    a:= g1.lat - g2.lat
    b:= g1.lon - g2.lon
    return math.Sqrt(a*a + b*b)
}

func interpolate(g1 gps, g2 gps, alpha float64) gps {
    var g gps

    g.lat = g1.lat * (1-alpha) + g2.lat * alpha
    g.lon = g1.lon * (1-alpha) + g2.lon * alpha

    return g
}

func latlon2meters(lat float64, lon float64) (float64, float64){
    resolution_lat := 1.0 / (111111.0) 
    resolution_lon := 1.0 / (111111.0 * math.Cos(lat / 360.0 * (3.1415926 * 2.0)))

    lat_meter := (lat - 40) / resolution_lat
    lon_meter := (lon + 70) / resolution_lon

    return lat_meter, lon_meter
}

func getInterceptionPoint(lat1 float64,lon1 float64, lat2 float64, lon2 float64, distance float64) (float64, float64) {
    A := (lat2 - lat1) * (lat2 - lat1) + 1.0 * (lon2 - lon1) * (lon2 - lon1)
    B := 2.0 * (lat2 - lat1) * lat1 + 1.0 * 2.0 * (lon2 - lon1) * lon1
    C := lat1 * lat1 + 1.0 * lon1 * lon1 - distance * distance

    R := B * B - 4*A*C

    alpha1 := -1.0
    alpha2 := -1.0

    if R > 0 {
        alpha1 = ( -B + math.Sqrt(R)) / (2.0 * A)
        alpha2 = ( -B - math.Sqrt(R)) / (2.0 * A)
    }

    return alpha1, alpha2
}

func getAngle(lat1 float64, lon1 float64, lat2 float64, lon2 float64) float64 {
    d_lat := lat2 - lat1 
    d_lon := (lon2 - lon1) * math.Cos(lat1 / 360.0 * (3.1415926 * 2.0))

    return math.Atan2(d_lat, d_lon) / (3.1415926 * 2.0) * 360.0
}


func main() {

    //fmt.Println(int(getAngle(42.0,71.0,43.0,71.0) + 180.0) % 360 )
    //fmt.Println(int(getAngle(42.0,71.0,41.0,71.0) + 180.0) % 360 )
    //fmt.Println(int(getAngle(42.0,71.0,42.0,72.0) + 180.0) % 360 )
    //fmt.Println(int(getAngle(42.0,71.0,42.0,70.0) + 180.0) % 360 )

    
    files, _ := ioutil.ReadDir(os.Args[1])

    var samplingRate int = 1 // Use this to simulate different sampling rate 
    var isEnableInterpolation int = 1 // Use this to turn on/off interpolation
    if len(os.Args)>3 {
        samplingRate,_ = strconv.Atoi(os.Args[3])
    }

    if len(os.Args)>4 {
        isEnableInterpolation,_ = strconv.Atoi(os.Args[4])
    }

    fmt.Println("SamplingRate: ",samplingRate)

    for _, f := range files {
        name := f.Name()
        trip_id_str := strings.Split(strings.Split(name, ".")[0],"_")[1]

        fmt.Println(os.Args[1]+f.Name()+"   trip ID: "+trip_id_str)
        file, _ := os.Open(os.Args[1]+f.Name())
        scanner := bufio.NewScanner(file)



        var gps_trace []gps
        var cc int = 0 
        for scanner.Scan() {
            line := scanner.Text()
            items := strings.Split(line,",")


            if len(items) < 4 {
                fmt.Println("Warning, incompleted line")
                continue
            }

            var loc gps
            loc.lat,_ = strconv.ParseFloat(items[1], 64)
            loc.lon,_ = strconv.ParseFloat(items[2], 64)
            loc.timestamp, _ = strconv.Atoi(items[3])
            
            if cc % samplingRate == 0 {
                gps_trace = append(gps_trace, loc)
            }

            cc = cc + 1
        }
        file.Close()

        for i:= 1; i < len(gps_trace); i ++ {
            gps_trace[i].timestamp -= gps_trace[0].timestamp
            gps_trace[i].tsFloat = float64(gps_trace[i].timestamp)
        } 

        gps_trace[0].timestamp = 0
        gps_trace[0].tsFloat = float64(gps_trace[0].timestamp)


        if len(gps_trace) == 0 {
            fmt.Println("Warning, Trip is too short")
            continue
        }

        var gps_trace_const []gps

        if isEnableInterpolation == 1 {
            cur_p := 0
            cur_alpha := 0.0
            tar_p := 0
            lat := gps_trace[0].lat
            lon := gps_trace[0].lon

            for {
                if tar_p == len(gps_trace) - 1 {
                    break
                }

                lat0, lon0 := latlon2meters(lat, lon)
                lat1, lon1 := latlon2meters(gps_trace[tar_p].lat, gps_trace[tar_p].lon) 
                lat2, lon2 := latlon2meters(gps_trace[tar_p+1].lat, gps_trace[tar_p+1].lon) 

                a1, a2 := getInterceptionPoint(lat1 - lat0, lon1 - lon0, lat2 - lat0, lon2 - lon0, 5.0)

                if (a1 < 0.0 || a1 > 1.0) && (a2 < 0.0 || a2 > 1.0){
                    tar_p = tar_p + 1
                } else {
                    if a1 >= 0.0 && a1 <= 1.0 {
                        if (tar_p == cur_p && a1 > cur_alpha) || (tar_p > cur_p) {
                            cur_p = tar_p
                            cur_alpha = a1
                            lat = (1-a1) * gps_trace[tar_p].lat + a1 * gps_trace[tar_p+1].lat
                            lon = (1-a1) * gps_trace[tar_p].lon + a1 * gps_trace[tar_p+1].lon

                            var loc gps
                            loc.lat = lat 
                            loc.lon = lon
                            loc.tsFloat = (1-a1) * gps_trace[tar_p].tsFloat + a1 * gps_trace[tar_p+1].tsFloat

                            gps_trace_const = append(gps_trace_const, loc)
                            continue
                        } else {
                            if a2 >= 0.0 && a2 <= 1.0 {
                                if (tar_p == cur_p && a2 > cur_alpha) || (tar_p > cur_p) {
                                    cur_p = tar_p
                                    cur_alpha = a2
                                    lat = (1-a2) * gps_trace[tar_p].lat + a2 * gps_trace[tar_p+1].lat
                                    lon = (1-a2) * gps_trace[tar_p].lon + a2 * gps_trace[tar_p+1].lon

                                    var loc gps
                                    loc.lat = lat 
                                    loc.lon = lon
                                    loc.tsFloat = (1-a2) * gps_trace[tar_p].tsFloat + a2 * gps_trace[tar_p+1].tsFloat

                                    gps_trace_const = append(gps_trace_const, loc)
                                    continue

                                }
                            }
                        }
                    }

                    tar_p = tar_p + 1

                }


            }




        } else {
            gps_trace_const = gps_trace
        }


        //

        if len(gps_trace_const) < 2 {
            continue
        } 



        var gps_trace_itp []gps

        for i:=0; i< len(gps_trace_const)-1; i++ {
            l:= distance(gps_trace_const[i], gps_trace_const[i+1])
            n:= int(math.Ceil(l/0.00001))
            

            gps_trace_itp = append(gps_trace_itp,gps_trace_const[i])
            gps_trace_itp[len(gps_trace_itp)-1].speedInv = gps_trace_const[i+1].tsFloat - gps_trace_const[i].tsFloat

            if n < 100 { // Remove those long tunnel
                n= 5
                for j:=1; j<n; j++ {
                    gps_trace_itp = append(gps_trace_itp,interpolate(gps_trace_const[i], gps_trace_const[i+1], float64(j)/float64(n)))
                    gps_trace_itp[len(gps_trace_itp)-1].speedInv = gps_trace_const[i+1].tsFloat - gps_trace_const[i].tsFloat
               }
            }
        }

        gps_trace_itp = append(gps_trace_itp, gps_trace_const[len(gps_trace_const)-1])
        gps_trace_itp[len(gps_trace_itp)-1].speedInv = 0.0


        //Add Length Info
        total_distance := 0.0

        gps_trace_itp[0].index = 0
        gps_trace_itp[0].total_distance = 0.0

        for i:=1; i< len(gps_trace_itp); i++ {
            total_distance = total_distance + distance(gps_trace_itp[i], gps_trace_itp[i-1])
            gps_trace_itp[i].total_distance = total_distance
            gps_trace_itp[i].index = i
        }


        if isEnableInterpolation == 0{
            gps_trace_itp = gps_trace
        }

        //Dump to File
        var angle float64
        angle = 0.0

        //for ind,v := range gps_trace_itp 
        for i:=0; i< len(gps_trace_itp); i++ {
            v:=gps_trace_itp[i]

            if i < len(gps_trace_itp)-1 {
                angle = getAngle(v.lat,v.lon, gps_trace_itp[i+1].lat, gps_trace_itp[i+1].lon)
            }

            nlat:=strconv.Itoa(int(math.Floor((v.lat / 0.001))))
            nlon:=strconv.Itoa(int(math.Floor((-v.lon / 0.001))))

            filename:= os.Args[2]+"/"+nlat+"_"+nlon+".txt"
            //fmt.Println(filename)
            f, _ := os.OpenFile(filename, os.O_APPEND | os.O_RDWR | os.O_CREATE, 0666) 

            //f.WriteString(fmt.Sprintf("%d %d %.6f %.6f %.12f\n",ind, v.index, v.lat, v.lon, v.total_distance))
            f.WriteString(fmt.Sprintf("%s %d %.6f %.6f %.12f %.6f %.2f\n",trip_id_str, v.index, v.lat, v.lon, v.total_distance, v.speedInv, angle))
            f.Close()
        }

    }

}

