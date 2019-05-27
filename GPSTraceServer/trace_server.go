package main

import "fmt"
//import "io/ioutil"
import "os"
import "bufio"
import "strings"
import "strconv"
import "math"
import "sync"
import "net"
import "time"
import "sort"
import "image/png"
import "image"
import "image/color"
import "bytes"
import "github.com/ajstarks/svgo"
//import "math/rand"
import "../GPSTraceServer/GaussianMixtureModel"
import "unsafe"
import "runtime"
import "runtime/debug"



var svgID int = 0
var debugMode int = 0


type gps struct{
	lat float64
	lon float64
	index int
	id    int
	speed float64
	total_distance float64
	clusterID int
	clusterDist float64
	ignored bool
	angle float64
}

type twoint struct {
	i1 int
	i2 int
}


func GPStoGMMPoint(in []gps) []GMM.Point {
	out := make([]GMM.Point, len(in))
	for i:=0; i<len(in); i++ {
		out[i].X = in[i].lat
		out[i].Y = in[i].lon
	}
	return out
}

func GMMPointtoGPS(in []GMM.Point ) []gps{
	out := make([]gps, len(in))
	for i:=0; i<len(in); i++ {
		out[i].lat = in[i].X
		out[i].lon = in[i].Y
	}
	return out
}


func latlonDistance(g1 gps, g2 gps) float64 {
	a:= g1.lat - g2.lat
	b:= (g1.lon - g2.lon) * math.Cos(g1.lat / 360.0 * 2.0 * 3.1415926)
	return math.Sqrt(a*a + b*b)
}


func min (a int, b int) int {
    if a>b {
        return b
    }

    return a
}




func KMean(obs []gps, K int) []gps {
	centroids := make([]gps, K)

	// Initialize Centroids
	for i := 0; i< K; i++ {
		centroids[i] = obs[i * 70667 % len(obs)]
	}

	iteration := 0
	for {

		update_num := 0
		iteration += 1

		
		// Assign Label
		for i := 0; i < len(obs); i ++ {
			min_dist := 10000000.0
			label := 0

			for j := 0; j < len(centroids) ; j++ {
				d := latlonDistance(obs[i], centroids[j])
				if d < min_dist {
					min_dist = d
					label = j
				}
			}

			if obs[i].clusterID != label{
				update_num += 1
			}


			obs[i].clusterID = label
			obs[i].clusterDist = min_dist
			

		}

		
		// Update
		for j := 0; j < len(centroids); j ++ {
			lat := 0.0
			lon := 0.0
			n := 0
			centroids[j].clusterDist = 0

			var dists []float64
			for i := 0; i < len(obs); i ++ {
				if obs[i].clusterID == j {
					dists = append(dists, obs[i].clusterDist)
				}
			}

			sort.Float64s(dists)

			threshold := 10000.0

			if len(dists) > 5 {
				threshold = dists[len(dists) - 1]
			}


			for i := 0; i < len(obs); i ++ {
				if obs[i].clusterID == j {
					if obs[i].clusterDist <= threshold {
						lat += obs[i].lat
						lon += obs[i].lon 
						centroids[j].clusterDist += obs[i].clusterDist
						n = n + 1
						obs[i].ignored = false
					} else {
						obs[i].ignored = true
					}
				}
			}

			if n > 0 {
				centroids[j].lat = lat / float64(n)
				centroids[j].lon = lon / float64(n)
				centroids[j].clusterDist = centroids[j].clusterDist / float64(n) + 0.00005
			}
		}

		if update_num == 0 || iteration > 1000{
			break
		}

	}

	return centroids
}


func BIC(obs []gps, centroids []gps, K int) float64 {
	var bic float64 = 0.0
	var theta float64 = 0.00050

	for i:=0; i< len(obs); i++ {
		if obs[i].ignored == false {
		//theta = math.Sqrt(centroids[obs[i].clusterID].clusterDist)
			bic += -2.0 * math.Log(1.0/(2.0*3.1415926*theta*theta)) + 2.0 * obs[i].clusterDist * obs[i].clusterDist / (2.0 * theta * theta) 
		}
	}

	bic += math.Log(float64(len(obs))) * float64(K)

	return bic
}


func XMean(obs []gps, K1 int, K2 int) []gps{
	var centroids []gps

	min_bic := 1000000000.0
	best_k := K1

	for k:=K1; k <= K2; k++ {
		centroids  := KMean(obs, k)

		bic := BIC(obs,centroids, k)

		fmt.Println(k, bic)

		if bic < min_bic {
			min_bic = bic
			best_k = k
		}	
	}
	centroids = KMean(obs, best_k)

	return centroids
}


func RunGMM(obs []gps, centerX float64, centerY float64, K1 int, K2 int) []gps{

	var center GMM.Point
	center.X = centerX
	center.Y = centerY

	var best_mean []GMM.Point
	//var best_covariances []GMM.Matrix2D
	//var best_log_likelihood float64
	//var best_gamma []float64

	var best_bic float64 = 100000000.0
	//var best_k = K1

	for k:=K1; k<=K2; k++ {
		//mean, covariances, log_likelihood, Gamma := GMM.GaussianMixtureModel_EM(GPStoGMMPoint(obs), k, center) 
		mean, _, log_likelihood, _ := GMM.GaussianMixtureModel_EM(GPStoGMMPoint(obs), k, center) 


		fmt.Println(k, math.Log(float64(len(obs)*2)) * float64(k) * (2.0 + 4.0 + 1.0) - 2.0 * log_likelihood, log_likelihood)
		if k== K1 {
			best_mean = mean
		}

		if math.Log(float64(len(obs) * 2)) * float64(k) * (2.0 + 4.0 + 1.0) - 2.0 * log_likelihood < best_bic {
			best_bic = math.Log(float64(len(obs) * 2)) * float64(k) * (2.0 + 4.0 + 1.0) - 2.0 * log_likelihood
			//best_k = k 
			best_mean = mean
			//best_covariances = covariances
			//best_log_likelihood = log_likelihood
			//best_gamma = Gamma
		}
	}


	return GMMPointtoGPS(best_mean)
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




type GPSList []gps

func (slice GPSList) Len() int {
	return len(slice)
}

func (slice GPSList) Less(i,j int) bool {
	return slice[i].index < slice[j].index
}

func (slice GPSList) Swap(i,j int) {
	slice[i], slice[j] = slice[j], slice[i]
}

type gpsPair struct{
	gA gps
	gB gps
}

func distance(g1 gps, g2 gps) float64 {
	a:= g1.lat - g2.lat
	b:= g1.lon - g2.lon
	return math.Sqrt(a*a + b*b)
}



type TraceDataset struct {
	cache map[string][]gps
	cache_timestamps map[int]string
	cache_size map[string]int64
	cache_timestamp_max int 
	cache_timestamp_min int
	cache_total_size int64 
	cache_max_size int64

	giveMemBacktoOSSize uint64 
	forceGCSize uint64

	path string
	mu	sync.RWMutex
	trip_bound int 
}

func MakeTraceDataset(path string, config string) *TraceDataset {
	TD := new(TraceDataset)
	TD.cache = make(map[string][]gps)
	TD.path = path
	TD.trip_bound, _ = strconv.Atoi(config)

	TD.cache_timestamp_max = 0
	TD.cache_timestamp_min = 0
	TD.cache_timestamps = make(map[int]string)
	TD.cache_size = make(map[string]int64)

	TD.cache_total_size = 0
	//TD.cache_max_size = 1024 * 1024 * 1024 * 8  // 8 GB
	TD.cache_max_size = 1024 * 1024 * 1024 * 3  // 4 GB
	TD.giveMemBacktoOSSize = 1024 * 1024 * 1024 * 8 // 8 GB
	TD.forceGCSize = 1024 * 1024 * 1024 * 12 // 12 GB

	// Max mem usage should be 4 GB + 12 GB + 8 GB = 24 GB  ...


	return TD
}

func makeTimestamp() int64 {
    return time.Now().UnixNano() / int64(time.Millisecond)
}

func (TD *TraceDataset)QuerySamples(lat float64, lon float64, radius float64) []gps {
	var result []gps

	var center_gps gps

	//radius = radius * 2.0

	radius_lon := radius * 1.5

	center_gps.lat = lat
	center_gps.lon = lon

	nlat_center:=int(math.Floor((lat / 0.001)))
    nlon_center:=int(math.Floor((-lon / 0.001)))

    local_lat := lat - float64(nlat_center) * 0.001
    local_lon := -lon - float64(nlon_center) * 0.001

    var region_list []int

    region_list = append(region_list, nlat_center)
    region_list = append(region_list, nlon_center)

    if local_lat < radius {
    	region_list = append(region_list, nlat_center-1)
    	region_list = append(region_list, nlon_center)
    }

    if local_lat > 0.001-radius {
    	region_list = append(region_list, nlat_center+1)
    	region_list = append(region_list, nlon_center)
    }

    if local_lon < radius {
    	region_list = append(region_list, nlat_center)
    	region_list = append(region_list, nlon_center-1)
    }

    if local_lon > 0.001-radius_lon {
    	region_list = append(region_list, nlat_center)
    	region_list = append(region_list, nlon_center+1)
    }

    if local_lat < radius && local_lon < radius {
    	region_list = append(region_list, nlat_center-1)
    	region_list = append(region_list, nlon_center-1)
    }


    if local_lat < radius && local_lon > 0.001-radius_lon {
    	region_list = append(region_list, nlat_center-1)
    	region_list = append(region_list, nlon_center+1)
    }

    if local_lat > 0.001-radius && local_lon < radius_lon {
    	region_list = append(region_list, nlat_center+1)
    	region_list = append(region_list, nlon_center-1)
    }

    if local_lat > 0.001-radius && local_lon > 0.001-radius_lon {
    	region_list = append(region_list, nlat_center+1)
    	region_list = append(region_list, nlon_center+1)
    }

    for i:=0; i<len(region_list); i = i+2 {
    	sign := strconv.Itoa(region_list[i])+"_"+strconv.Itoa(region_list[i+1])

    	TD.mu.RLock()
    		_,ok:=TD.cache[sign]
    		if ok == true {
    			for _,loc := range TD.cache[sign] {
    				if distance(loc, center_gps) < radius {
        				result = append(result, loc)
        			}
    			}
    		}

    	TD.mu.RUnlock()

    	if ok == false {
    		
    		TD.mu.Lock()

    		_,ok=TD.cache[sign]
    		if ok == true {
    			for _,loc := range TD.cache[sign] {
    				if distance(loc, center_gps) < radius {
        				result = append(result, loc)
        			}
    			}
    		}

    		if ok == false {
    			//Load from disk
    			filename:= TD.path+"/"+sign+".txt"
    			file, _ := os.Open(filename)
				scanner := bufio.NewScanner(file)

				TD.cache[sign] = []gps{}

    			for scanner.Scan() {
        			line := scanner.Text()
        			items := strings.Split(line," ")

        			if len(items)<6 {
        				continue
        			}


        			var loc gps
        			loc.lat,_ = strconv.ParseFloat(items[2], 64)
        			loc.lon,_ = strconv.ParseFloat(items[3], 64)
        			loc.id,_ = strconv.Atoi(items[0])
        			loc.index,_ = strconv.Atoi(items[1])
        			loc.total_distance,_ = strconv.ParseFloat(items[4], 64) 
        			if len(items) == 6 {
        				loc.speed,_ = strconv.ParseFloat(strings.Split(items[5],"\n")[0],64)
        				loc.angle = 0.0
        			}
        			if len(items) == 7 {
        				loc.speed,_ = strconv.ParseFloat(items[5],64)
        				loc.angle,_ = strconv.ParseFloat(strings.Split(items[6],"\n")[0],64)
        			}	

        			if loc.id > TD.trip_bound {
        				continue
        			}

        			//fmt.Println(loc.total_distance)
        			if latlonDistance(loc, center_gps) < radius {
        				result = append(result, loc)
        				//fmt.Println(loc.lat,loc.lon,distance(loc, center_gps))

        			}
        			TD.cache[sign] = append(TD.cache[sign], loc)

        		}
        		file.Close()

	        	TD.cache_timestamps[TD.cache_timestamp_max] = sign 
	        	TD.cache_timestamp_max = TD.cache_timestamp_max + 1

	        	var tmpgps gps

	        	TD.cache_size[sign] = int64(unsafe.Sizeof(tmpgps))*int64(len(TD.cache[sign]))
	        	TD.cache_total_size += TD.cache_size[sign]

	        	// Move out one items in the cache if the cache is full
	        	for {
	        		if TD.cache_total_size > TD.cache_max_size  && TD.cache_timestamp_max - TD.cache_timestamp_min > 5{

	        			TD.cache_total_size -= TD.cache_size[TD.cache_timestamps[TD.cache_timestamp_min]]
	        			delete(TD.cache, TD.cache_timestamps[TD.cache_timestamp_min])
	        			delete(TD.cache_size, TD.cache_timestamps[TD.cache_timestamp_min])
	        			delete(TD.cache_timestamps, TD.cache_timestamp_min)
	        			TD.cache_timestamp_min += 1
	        			// GC ? 

	        		} else {
	        			break
	        		}
	        	}
	        	var m runtime.MemStats
		        runtime.ReadMemStats(&m)

		        if m.HeapIdle - m.HeapReleased > TD.giveMemBacktoOSSize {
		        	fmt.Println("FreeOSMemory")
		        	debug.FreeOSMemory()
		        }

		        if m.Alloc > TD.forceGCSize {
		        	fmt.Println("Force GC")
		        	runtime.GC()
		        }

		        fmt.Println("Cache Miss ", sign, " Size: ", TD.cache_size[sign], " bytes  Total Cache Size:",  TD.cache_total_size / 1024 / 1024 , " MB", "Heap Alloc ", m.Alloc / 1024 / 1024, " MB   Unused", (m.HeapIdle - m.HeapReleased) / 1024 / 1024, " MB")
		        	
	        	//fmt.Println("Cache Miss ", sign, " Size: ", TD.cache_size[sign], " bytes  Total Cache Size:",  TD.cache_total_size / 1024 / 1024 , " MB")
	        
        	}

        	TD.mu.Unlock()


        }

    }

    return result

}


func (TD *TraceDataset)QuerySamplesRect(lat_min float64, lon_min float64, lat_max float64, lon_max float64) []gps {
	var result []gps

	//var center_gps gps
	//center_gps.lat = lat
	//center_gps.lon = lon

	nlat_center_min:=int(math.Floor((lat_min / 0.001)))
    nlon_center_min:=int(math.Floor((-lon_min / 0.001)))
    nlat_center_max:=int(math.Floor((lat_max / 0.001)))
    nlon_center_max:=int(math.Floor((-lon_max / 0.001)))


    var region_list []int

    for i:= nlat_center_min; i<= nlat_center_max; i++ {
    	for j:= nlon_center_max; j<= nlon_center_min; j++ {
    		region_list = append(region_list, i)
    		region_list = append(region_list, j)
    	}
    }
    

    for i:=0; i<len(region_list); i = i+2 {
    	sign := strconv.Itoa(region_list[i])+"_"+strconv.Itoa(region_list[i+1])

    	TD.mu.RLock()
    		_,ok:=TD.cache[sign]
    		if ok == true {
    			for _,loc := range TD.cache[sign] {
    				//if distance(loc, center_gps) < radius {
    				//fmt.Println("Cache Hit", len(result))
        			result = append(result, loc)
        			//}
    			}
    		}

    	TD.mu.RUnlock()

    	if ok == false {
    		//fmt.Println("Cache Miss ", sign)
    		TD.mu.Lock()

    			_,ok=TD.cache[sign]
	    		if ok == true {
	    			for _,loc := range TD.cache[sign] {
	    				
	        				result = append(result, loc)
	        			
	    			}
	    		}

	    		if ok == false {

	    			//Load from disk
	    			filename:= TD.path+"/"+sign+".txt"
	    			file, _ := os.Open(filename)
					scanner := bufio.NewScanner(file)
					//fmt.Println("filename ", filename)
					TD.cache[sign] = []gps{}

	    			for scanner.Scan() {
	        			line := scanner.Text()
	        			items := strings.Split(line," ")

	        			if len(items)<5 {
	        				continue
	        			}


	        			var loc gps
	        			loc.lat,_ = strconv.ParseFloat(items[2], 64)
	        			loc.lon,_ = strconv.ParseFloat(items[3], 64)
	        			loc.id,_ = strconv.Atoi(items[0])
	        			loc.index,_ = strconv.Atoi(items[1])
	        			loc.total_distance,_ = strconv.ParseFloat(strings.Split(items[4],"\n")[0],64)
	        			if len(items) == 6 {
        					loc.speed,_ = strconv.ParseFloat(strings.Split(items[5],"\n")[0],64)
        					loc.angle = 0.0
	        			}
	        			if len(items) == 7 {
	        				loc.speed,_ = strconv.ParseFloat(items[5],64)
	        				loc.angle,_ = strconv.ParseFloat(strings.Split(items[6],"\n")[0],64)
	        			}	
	        			//fmt.Println(loc.total_distance)
	        			//if distance(loc, center_gps) < radius {
	        			//fmt.Println("Cache Hit", len(result))
	        			result = append(result, loc)
	        				//fmt.Println(loc.lat,loc.lon,distance(loc, center_gps))

	        			//}
	        			TD.cache[sign] = append(TD.cache[sign], loc)

	        		}
	        		file.Close()

	        		TD.cache_timestamps[TD.cache_timestamp_max] = sign 
		        	TD.cache_timestamp_max = TD.cache_timestamp_max + 1

		        	var tmpgps gps

		        	TD.cache_size[sign] = int64(unsafe.Sizeof(tmpgps))*int64(len(TD.cache[sign]))
		        	TD.cache_total_size += TD.cache_size[sign]

		        	// Move out one items in the cache if the cache is full
		        	for {
		        		// var m runtime.MemStats
		        		// runtime.ReadMemStats(&m)

		        		//if (TD.cache_total_size > TD.cache_max_size || m.Alloc > uint64(TD.cache_max_size))  && TD.cache_timestamp_max - TD.cache_timestamp_min > 5{
		        		if TD.cache_total_size > TD.cache_max_size  && TD.cache_timestamp_max - TD.cache_timestamp_min > 5{

		        			

		        			TD.cache_total_size -= TD.cache_size[TD.cache_timestamps[TD.cache_timestamp_min]]
		        			delete(TD.cache, TD.cache_timestamps[TD.cache_timestamp_min])
		        			delete(TD.cache_size, TD.cache_timestamps[TD.cache_timestamp_min])
		        			delete(TD.cache_timestamps, TD.cache_timestamp_min)
		        			TD.cache_timestamp_min += 1
		        			// GC ? 

		        			// if m.Alloc > uint64(TD.cache_max_size) {
		        			// 	runtime.GC() // Force a GC
		        			// }



		        		} else {
		        			break
		        		}
		        	}

		        	var m runtime.MemStats
		        	runtime.ReadMemStats(&m)

		        	if m.HeapIdle - m.HeapReleased > TD.giveMemBacktoOSSize {
		        		fmt.Println("FreeOSMemory")
		        		debug.FreeOSMemory()
		        	}

		        	if m.Alloc > TD.forceGCSize {
		        	fmt.Println("Force GC")
		        	runtime.GC()
			        }




		        	//fmt.Println("Cache Miss ", sign, " Size: ", TD.cache_size[sign], " bytes  Total Cache Size:",  TD.cache_total_size / 1024 / 1024 , " MB", "Heap Alloc ", m.Alloc / 1024 / 1024, " MB")
		        	//fmt.Println("Cache Miss ", sign, " Size: ", TD.cache_size[sign], " bytes  Total Cache Size:",  TD.cache_total_size / 1024 / 1024 , " MB")
		        	fmt.Println("Cache Miss ", sign, " Size: ", TD.cache_size[sign], " bytes  Total Cache Size:",  TD.cache_total_size / 1024 / 1024 , " MB", "Heap Alloc ", m.Alloc / 1024 / 1024, " MB  Unused", (m.HeapIdle - m.HeapReleased) / 1024 / 1024, " MB")
		        
		        	


		        	

		        	// if m.Alloc / 1024 / 1024 > uint64(TD.cache_max_size) * 3 {
		        	// 	runtime.GC()
		        	// 	runtime.ReadMemStats(&m)
		        	// 	fmt.Println("Force GC, Heap Alloc ", m.Alloc / 1024 / 1024, " MB")

		        	// }


			    }   


        	TD.mu.Unlock()
        }

    }

    return result

}





type QueryReply struct {
	result string
	detailResult string
	id int
}


func Query(lat1 float64, lon1 float64, lat2 float64, lon2 float64, radius float64, id int, replychan chan QueryReply,  dataset *TraceDataset) {
	var g1 gps
	var g2 gps

	g1.lat = lat1
	g1.lon = lon1

	g2.lat = lat2
	g2.lon = lon2

	gps_list1 := dataset.QuerySamples(g1.lat, g1.lon, radius)
	gps_list2 := dataset.QuerySamples(g2.lat, g2.lon, radius)

	fmt.Println(len(gps_list1), len(gps_list2))

	gpsPairs1 := make(map[int]gpsPair)
	gpsPairs2 := make(map[int]gpsPair)

	var gACounter int = 0
	var gBCounter int = 0
	var gUnionCounter int = 0
	var ValidCounterDir1 int = 0
	var ValidCounterDir2 int = 0

	for _,v := range gps_list1 {
		if _,ok:=gpsPairs1[v.id]; ok {

			if v.index < gpsPairs1[v.id].gA.index {
				var pair gpsPair = gpsPairs1[v.id]
				pair.gA = v
				gpsPairs1[v.id] = pair
			}

			if v.index > gpsPairs1[v.id].gB.index {
				var pair gpsPair = gpsPairs1[v.id]
				pair.gB = v
				gpsPairs1[v.id] = pair
			}

		} else {
			var pair gpsPair 
			pair.gA = v
			pair.gB = v
			gpsPairs1[v.id] = pair
			gACounter = gACounter + 1
		}
	}

	for _,v := range gps_list2 {
		if _,ok:=gpsPairs2[v.id]; ok {

			if v.index < gpsPairs2[v.id].gA.index {
				var pair gpsPair = gpsPairs2[v.id]
				pair.gA = v
				gpsPairs2[v.id] = pair
			}

			if v.index > gpsPairs2[v.id].gB.index {
				var pair gpsPair = gpsPairs2[v.id]
				pair.gB = v
				gpsPairs2[v.id] = pair
			}

		} else {
			var pair gpsPair 
			pair.gA = v
			pair.gB = v
			gpsPairs2[v.id] = pair
			gBCounter = gBCounter + 1
		}
	}


	dist := latlonDistance(g1,g2)


	for k,v1 := range gpsPairs1 {
		if v2,ok := gpsPairs2[k]; ok {
			gUnionCounter = gUnionCounter + 1

			if dist <= radius+radius {
				ValidCounterDir1 = ValidCounterDir1 + 1
				ValidCounterDir2 = ValidCounterDir2 + 1
			}


			if v1.gB.index < v2.gA.index {
				//d:=v2.gA.total_distance - v1.gB.total_distance
				d:=float64(v2.gA.index - v1.gB.index) * 0.00001
				if d > dist-radius*2 && d < dist*2.0+radius*2 {
					ValidCounterDir1 = ValidCounterDir1 + 1
				}
			} 

			if v1.gA.index > v2.gB.index {
				//d:=v1.gA.total_distance - v2.gB.total_distance
				d:=float64(v1.gA.index - v2.gB.index) * 0.00001
				if d > dist-radius*2 && d < dist*2.0+radius*2 {
					ValidCounterDir2 = ValidCounterDir2 + 1
				}
			}


		}
	}

	var reply QueryReply

	reply.result = fmt.Sprintf("%d %d %d %d %d ",gACounter, gBCounter, gUnionCounter, ValidCounterDir1, ValidCounterDir2)
	reply.id = id

	replychan <- reply

}

func QueryValidTrace(lat1 float64, lon1 float64, lat2 float64, lon2 float64, radius1 float64, radius2 float64, dist float64, id int,  dataset *TraceDataset) map[int]twoint {
	var g1 gps
	var g2 gps

	g1.lat = lat1
	g1.lon = lon1

	g2.lat = lat2
	g2.lon = lon2

	gps_list1 := dataset.QuerySamples(g1.lat, g1.lon, radius1)
	gps_list2 := dataset.QuerySamples(g2.lat, g2.lon, radius2)

	//fmt.Println("aaa", g1,g2,len(gps_list1), len(gps_list2))

	gpsPairs1 := make(map[int]gpsPair)
	gpsPairs2 := make(map[int]gpsPair)

	var gACounter int = 0
	var gBCounter int = 0
	var gUnionCounter int = 0
	var ValidCounterDir1 int = 0
	var ValidCounterDir2 int = 0



	for _,v := range gps_list1 {
		if _,ok:=gpsPairs1[v.id]; ok {

			if v.index < gpsPairs1[v.id].gA.index {
				var pair gpsPair = gpsPairs1[v.id]
				pair.gA = v
				gpsPairs1[v.id] = pair
			}

			if v.index > gpsPairs1[v.id].gB.index {
				var pair gpsPair = gpsPairs1[v.id]
				pair.gB = v
				gpsPairs1[v.id] = pair
			}

		} else {
			var pair gpsPair 
			pair.gA = v
			pair.gB = v
			gpsPairs1[v.id] = pair
			gACounter = gACounter + 1
		}
	}

	for _,v := range gps_list2 {
		if _,ok:=gpsPairs2[v.id]; ok {

			if v.index < gpsPairs2[v.id].gA.index {
				var pair gpsPair = gpsPairs2[v.id]
				pair.gA = v
				gpsPairs2[v.id] = pair
			}

			if v.index > gpsPairs2[v.id].gB.index {
				var pair gpsPair = gpsPairs2[v.id]
				pair.gB = v
				gpsPairs2[v.id] = pair
			}

		} else {
			var pair gpsPair 
			pair.gA = v
			pair.gB = v
			gpsPairs2[v.id] = pair
			gBCounter = gBCounter + 1
		}
	}


	// 
	for k,v := range gpsPairs1 {
		if v.gB.index - v.gA.index > 100 {
			delete(gpsPairs1,k)
		} 
	}

	for k,v := range gpsPairs2 {
		if v.gB.index - v.gA.index > 100 {
			delete(gpsPairs2,k)
		} 
	}



	//dist := distance(g1,g2)
	if dist < 0.0 {
		dist = latlonDistance(g1, g2)
	}

	//fmt.Println("dist", dist, latlonDistance(g1, g2))

	result := make(map[int]twoint)

	//fmt.Println("aaa gpsPairs", len(gpsPairs1), len(gpsPairs2))
	for k,v1 := range gpsPairs1 {
		if v2,ok := gpsPairs2[k]; ok {
			gUnionCounter = gUnionCounter + 1

			// d can be less than 0  
			// if dist <= radius1+radius2 {
			if latlonDistance(g1, g2) <= radius1+radius2 {
				var r twoint
				r.i1 = (v1.gB.index + v1.gA.index) / 2
				r.i2 = (v2.gB.index + v2.gA.index) / 2
				result[v1.gB.id] = r //(v2.gB.index + v2.gA.index) / 2 //rand.Int() % (v2.gB.index - v2.gA.index+1) + v2.gA.index // (v2.gB.index + v2.gA.index) / 2 
			}

			if v1.gB.index <= v2.gA.index {
				//d:=v2.gA.total_distance - v1.gB.total_distance
				d:=float64(v2.gA.index - v1.gB.index) * 0.00001



				//if d > dist-radius*2 && d < dist*1.4+radius*2 {
				if d >= 0 && d < dist*2.0+radius1+radius2 {
					//fmt.Println("d-->", d, dist*1.4+radius*2)
					ValidCounterDir1 = ValidCounterDir1 + 1
					var r twoint
					r.i1 = (v1.gB.index + v1.gA.index) / 2
					r.i2 = (v2.gB.index + v2.gA.index) / 2
					//result[v1.gB.id] =  (v2.gB.index + v2.gA.index) / 2  //rand.Int() % (v2.gB.index - v2.gA.index+1) + v2.gA.index 
					result[v1.gB.id] =  r //rand.Int() % (v2.gB.index - v2.gA.index+1) + v2.gA.index 
				}
			} 

			if v1.gA.index >= v2.gB.index {
				//d:=v1.gA.total_distance - v2.gB.total_distance
				d:=float64(v1.gA.index - v2.gB.index) * 0.00001
				
				if d >= 0 && d < dist*2.0+radius1+radius2 {
					ValidCounterDir2 = ValidCounterDir2 + 1
				}
			}


		}
	}



	return result

}

func Query2Fast(lat1 float64, lon1 float64, lat2 float64, lon2 float64, lat3 float64, lon3 float64, radius1 float64,radius2 float64,radius3 float64, dist1 float64, dist2 float64, id int, replychan chan QueryReply,  dataset *TraceDataset, list1 map[int]twoint) {
	//list1 := QueryValidTrace(lat1,lon1,lat2,lon2,radius1, radius2, id,dataset)
	list2 := QueryValidTrace(lat2,lon2,lat3,lon3,radius2, radius3, dist2, id,dataset)

	//fmt.Println(len(list1), len(list2))


	var detail string
	detail = ""

	counter := 0
	for k, v := range list1 {
		detail = detail + fmt.Sprintf("%d %d ",k, v.i2)
		if _,ok := list2[k]; ok {
			counter = counter + 1
			
		}
	}

	var reply QueryReply

	reply.result = fmt.Sprintf("%d ",counter)
	reply.detailResult = detail
	reply.id = id

	replychan <- reply
}


func Query2(lat1 float64, lon1 float64, lat2 float64, lon2 float64, lat3 float64, lon3 float64, radius1 float64,radius2 float64,radius3 float64, dist1 float64, dist2 float64, id int, replychan chan QueryReply,  dataset *TraceDataset) {
	list1 := QueryValidTrace(lat1,lon1,lat2,lon2,radius1, radius2, dist1, id,dataset)
	list2 := QueryValidTrace(lat2,lon2,lat3,lon3,radius2, radius3, dist2, id,dataset)

	//fmt.Println(len(list1), len(list2))


	var detail string
	detail = ""

	counter := 0
	for k, v := range list1 {
		detail = detail + fmt.Sprintf("%d %d ",k, v.i2)
		if _,ok := list2[k]; ok {
			counter = counter + 1
			
		}
	}

	var reply QueryReply

	reply.result = fmt.Sprintf("%d ",counter)
	reply.detailResult = detail
	reply.id = id

	replychan <- reply
}


func Query2Interpolation(lat1 float64, lon1 float64, lat2 float64, lon2 float64, lat3 float64, lon3 float64, radius1 float64,radius2 float64,radius3 float64, dist1 float64,dist2 float64, id int, replychan chan QueryReply,  dataset *TraceDataset) {
	list1 := QueryValidTrace(lat1,lon1,lat2,lon2,radius1, radius2, dist1, id,dataset)
	list2 := QueryValidTrace(lat2,lon2,lat3,lon3,radius2, radius3, dist2, id,dataset)

	//fmt.Println(len(list1), len(list2))


	var detail string
	detail = ""

	counter := 0
	for k, _ := range list1 {
		
		if v2,ok := list2[k]; ok {
			counter = counter + 1
			detail = detail + fmt.Sprintf("%d %d %d ",k, v2.i1, v2.i2)
		}
	}

	// Load Trips and do interpolation!

	

	tripids := strings.Split(detail, " ")

	numOfTrips := len(tripids) / 3

	fmt.Println(numOfTrips)
	var result string = "0 0 "
	if numOfTrips != 0 {

		allTrips := make([][]gps, numOfTrips)

		minimumLength := 10000

	    for tid:=0; tid < numOfTrips *3; tid += 3 {
	    	tripid, _ := strconv.Atoi(tripids[tid])
	    	trip := loadTrip(os.Args[2]+"trip_"+tripids[tid]+".txt", tripid)

	    	trip_start_index, _ := strconv.Atoi(tripids[tid+1])
	    	trip_stop_index, _ := strconv.Atoi(tripids[tid+2])



	    	//fmt.Println(trip_start_index, trip_stop_index, len(trip), os.Args[2]+"trip_"+tripids[tid]+".txt")
	    	
	    	// change the step size from 25 to 8 (7.5)
	    	//for i:= trip_start_index + 25; i < trip_stop_index; i += 25 {
	    	for i:= trip_start_index + 8; i < trip_stop_index; i += 8 {
	    		allTrips[tid/3] = append(allTrips[tid/3], trip[i])
	    	}

	    	if len(allTrips[tid/3]) < minimumLength {
	    		minimumLength = len(allTrips[tid/3])
	    	}
	    }

	    

	    for i := 0; i< minimumLength; i ++ {
	    	lat:=0.0
	    	lon:=0.0

	    	for j:=0; j< numOfTrips; j ++ {
	    		lat += allTrips[j][i].lat
	    		lon += allTrips[j][i].lon
	    	}

	    	lat /= float64(numOfTrips) 
	    	lon /= float64(numOfTrips)

	    	result += fmt.Sprintf("%f %f ",lat, lon)
	    }

	}


	var reply QueryReply

	reply.result = result
	reply.detailResult = detail
	reply.id = id

	replychan <- reply
}



func QuerySpeed(lat1 float64, lon1 float64, radius1 float64, ranking float64, id int, replychan chan QueryReply, dataset *TraceDataset) {
	gpslist := dataset.QuerySamples(lat1, lon1, radius1)
	var reply QueryReply

	if len(gpslist) < 5 {
		

		reply.result = "100 "
		reply.id = id

		replychan <- reply

		return 
	}


	var speeds []float64

	for i:=0; i< len(gpslist); i ++ {
		if gpslist[i].speed > 0.07 {
			speeds = append(speeds, gpslist[i].speed)
		}
	}

	if len(speeds) < 5 {
		

		reply.result = "100 "
		reply.id = id

		replychan <- reply

		return 
	}


	rank := int(ranking * float64(len(speeds)))
	if rank > len(speeds) - 1 {
		rank = len(speeds) - 1
	}

	sort.Float64s(speeds)

	
	reply.result = fmt.Sprintf("%f ",speeds[rank])
	reply.id = id

	replychan <- reply



}



func handleConnectionP2P(conn net.Conn, dataset *TraceDataset) {
	buf := make([]byte, 4096)

	n,err := conn.Read(buf)
	data := string(buf[:n])
	fmt.Println(data)

	if err==nil {
		var g1 gps
		var g2 gps

		items := strings.Split(data,",")

		m := int(math.Floor(float64(len(items)) / 5.0))

		replies := make(chan QueryReply, m)

		ts1 := makeTimestamp()

		for i:=0; i<m; i++ {

			g1.lat,_ = strconv.ParseFloat(items[i*5+0], 64)
			g1.lon,_ = strconv.ParseFloat(items[i*5+1], 64)

			g2.lat,_ = strconv.ParseFloat(items[i*5+2], 64)
			g2.lon,_ = strconv.ParseFloat(items[i*5+3], 64)

			radius,_ := strconv.ParseFloat(items[i*5+4], 64)

			go Query(g1.lat, g1.lon, g2.lat, g2.lon, radius, i, replies, dataset)  

		}

		answermap := make(map[int]string)

		for i:=0; i<m; i++ {
			r := <- replies
			answermap[r.id] = r.result
		}

		var answer string = ""

		for i:=0; i<m; i++ {
			answer = answer + answermap[i]
		}

		fmt.Println("Return Size ", len(answer))
		fmt.Println("Time ", makeTimestamp() - ts1, "ms")

		conn.Write([]byte(answer))



	}



	conn.Close()
}
func SendLargeData(conn net.Conn, data string) {
	bytes := []byte(data)
	length := len(bytes)

	chunksize := 1024
	chunknum := int(math.Ceil(float64(length) / float64(chunksize)))

	firstMsg := fmt.Sprintf("ChunkNum %d ", chunknum)

	conn.Write([]byte(firstMsg))

	buf := make([]byte, 8192)

	_,_ = conn.Read(buf)


	for i:=0; i<chunknum; i++ {
		end := i*chunksize + chunksize
		if end > length{
			end = length
		}

		conn.Write(bytes[i*chunksize:end])
	}

}

func SendLargeDataBytes(conn net.Conn, data []byte) {
	bytes := data
	length := len(bytes)

	chunksize := 1024
	chunknum := int(math.Ceil(float64(length) / float64(chunksize)))

	firstMsg := fmt.Sprintf("ChunkNum %d ", chunknum)

	conn.Write([]byte(firstMsg))

	buf := make([]byte, 8192)

	_,_ = conn.Read(buf)


	for i:=0; i<chunknum; i++ {
		end := i*chunksize + chunksize
		if end > length{
			end = length
		}

		conn.Write(bytes[i*chunksize:end])
	}

}



func loadTrip(filename string, tid int) []gps{

	file, _ := os.Open(filename)
    scanner := bufio.NewScanner(file)
    var gps_trace []gps
        for scanner.Scan() {
            line := scanner.Text()
            items := strings.Split(line,",")

            var loc gps
            loc.lat,_ = strconv.ParseFloat(items[1], 64)
            loc.lon,_ = strconv.ParseFloat(items[2], 64)

            gps_trace = append(gps_trace, loc)
        }
        file.Close()


        if len(gps_trace) == 0 {
        	return gps_trace
        }

        var gps_trace_const []gps

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
                                gps_trace_const = append(gps_trace_const, loc)
                                continue

                            }
                        }
                    }
                }

                tar_p = tar_p + 1

            }


        }

        if len(gps_trace_const) < 2 {
            return gps_trace_const
        } 

        var gps_trace_itp []gps

        for i:=0; i< len(gps_trace_const)-1; i++ {
            l:= distance(gps_trace_const[i], gps_trace_const[i+1])
            n:= int(math.Ceil(l/0.00001))
            gps_trace_itp = append(gps_trace_itp,gps_trace_const[i])
            if n < 100 { // Remove those long tunnel
            	n = 5
                for j:=1; j<n; j++ {
                   gps_trace_itp = append(gps_trace_itp,interpolate(gps_trace_const[i], gps_trace_const[i+1], float64(j)/float64(n)))
               }
            }
        }

        gps_trace_itp = append(gps_trace_itp, gps_trace_const[len(gps_trace_const)-1])

        //Add Length Info
        total_distance := 0.0

        gps_trace_itp[0].index = 0
        gps_trace_itp[0].total_distance = 0.0

        for i:=1; i< len(gps_trace_itp); i++ {
            total_distance = total_distance + distance(gps_trace_itp[i], gps_trace_itp[i-1])
            gps_trace_itp[i].total_distance = total_distance
            gps_trace_itp[i].index = i
            gps_trace_itp[i].id = tid
        }


    return gps_trace_itp

}

func renderSVG(trip_id string, lat float64, lon float64, clat1 float64, clon1 float64, clat2 float64, clon2 float64, r1 float64, r2 float64) {
	f, _:= os.Create("trips"+strconv.Itoa(svgID)+".svg")
	//åsvgID += 1

	region := make([]float64, 4)

	region[0] = lat - 0.0006
	region[1] = lon - 0.001
	region[2] = lat + 0.0006
	region[3] = lon + 0.001

	sizex := 2048
	sizey := int(float64(sizex) / ((region[3] - region[1])*math.Cos(region[0]/360.0 * 3.1415926 * 2.0)) * (region[2] - region[0]) )

	canvas := svg.New(f)
    canvas.Start(sizex, sizey)


    tripids := strings.Split(trip_id, " ")


    for tid:=0; tid < len(tripids)-2; tid += 2 {
    	tripid, _ := strconv.Atoi(tripids[tid])
    	trip := loadTrip(os.Args[2]+"trip_"+tripids[tid]+".txt", tripid)

    	trip_target_index, _ := strconv.Atoi(tripids[tid+1])

    	for i := 0; i< len(trip)-10; i+=10 {

    		if i < trip_target_index - 300 || i > trip_target_index + 600 {
    			continue
    		}

    		c1lat := trip[i].lat
    		c1lon := trip[i].lon
    		c2lat := trip[i+10].lat
    		c2lon := trip[i+10].lon

    		ilat1:= int((1.0 - (c1lat - region[0]) / (region[2] - region[0])) * float64(sizey))
	        ilon1:= int((c1lon - region[1]) / (region[3] - region[1]) * float64(sizex))

	        ilat2:= int((1.0 - (c2lat - region[0]) / (region[2] - region[0])) * float64(sizey))
	        ilon2:= int((c2lon - region[1]) / (region[3] - region[1]) * float64(sizex))


	        if ilat1 < 0 || ilat2 < 0 || ilon1 < 0 || ilon2 < 0 {
	        	continue
	        }

	        if ilat1 > sizey || ilat2 > sizey || ilon1 > sizex || ilon2 > sizex {
	        	continue
	        }


	        //color := strconv.Itoa(int((i - trip_target_index) * 255 / 600))
	        color := "0"

	        canvas.Line(ilon1, ilat1, ilon2, ilat2, "stroke-width=\"1\" stroke-opacity=\"0.05\" stroke=\"rgba(255, "+color+","+color+",10)\"")
            //canvas.Circle(ilon1, ilat1, 2, "fill=\"none\" stroke-width=\"1\" stroke=\"rgba("+color+",255,"+color+", 10)\"")
            //canvas.Circle(ilon2, ilat2, 2, "fill=\"none\" stroke-width=\"1\" stroke=\"rgba("+color+",255,"+color+", 10)\"")

    	}
    }

    iclat1:= int((1.0 - (clat1 - region[0]) / (region[2] - region[0])) * float64(sizey))
	iclon1:= int((clon1- region[1]) / (region[3] - region[1]) * float64(sizex))

	iclat2:= int((1.0 - (clat2 - region[0]) / (region[2] - region[0])) * float64(sizey))
	iclon2:= int((clon2- region[1]) / (region[3] - region[1]) * float64(sizex))

    canvas.Circle(iclon1, iclat1, int(r1/0.006 * 1024.0), "fill=\"none\" stroke-width=\"1\" stroke=\"rgba(0,0,255, 10)\"")
    canvas.Circle(iclon2, iclat2, int(r2/0.006 * 1024.0), "fill=\"none\" stroke-width=\"1\" stroke=\"rgba(0,0,255, 10)\"")



    canvas.End()

}

func renderSamples(trip_id string, lat float64, lon float64, clat1 float64, clon1 float64, clat2 float64, clon2 float64, r1 float64, r2 float64) {
	f, _:= os.Create("samples"+strconv.Itoa(svgID)+".svg")
	
	region := make([]float64, 4)

	region[0] = lat - 0.0006
	region[1] = lon - 0.001
	region[2] = lat + 0.0006
	region[3] = lon + 0.001

	sizex := 2048
	sizey := int(float64(sizex) / ((region[3] - region[1])*math.Cos(region[0]/360.0 * 3.1415926 * 2.0)) * (region[2] - region[0]) )

	canvas := svg.New(f)
    canvas.Start(sizex, sizey)


    tripids := strings.Split(trip_id, " ")



    all_trips := make([][]gps, 35)

    local_tid := 0
    for tid:=0; tid < len(tripids)-2; tid += 2 {

    	tripid, _ := strconv.Atoi(tripids[tid])
    	trip := loadTrip(os.Args[2]+"trip_"+tripids[tid]+".txt", tripid)

    	trip_target_index, _ := strconv.Atoi(tripids[tid+1])


    	for i := trip_target_index - 300; i < trip_target_index + 900; i += 25 {
    		if i < 0 {
    			continue
    		}

    		if i >= len(trip) - 25 {
    			break
    		}

    		if i > trip_target_index {
    			all_trips[(i - trip_target_index) / 25 - 1] = append(all_trips[(i - trip_target_index) / 25 - 1], trip[i])

    		}

    	// for i := 0; i< len(trip)-25; i+=25 {

    	// 	if i < trip_target_index - 300 || i > trip_target_index + 900 {
    	// 		continue
    	// 	}

    		c1lat := trip[i].lat
    		c1lon := trip[i].lon
    		c2lat := trip[i+25].lat
    		c2lon := trip[i+25].lon

    		ilat1:= int((1.0 - (c1lat - region[0]) / (region[2] - region[0])) * float64(sizey))
	        ilon1:= int((c1lon - region[1]) / (region[3] - region[1]) * float64(sizex))

	        ilat2:= int((1.0 - (c2lat - region[0]) / (region[2] - region[0])) * float64(sizey))
	        ilon2:= int((c2lon - region[1]) / (region[3] - region[1]) * float64(sizex))


	        if ilat1 < 0 || ilat2 < 0 || ilon1 < 0 || ilon2 < 0 {
	        	continue
	        }

	        if ilat1 > sizey || ilat2 > sizey || ilon1 > sizex || ilon2 > sizex {
	        	continue
	        }




	        // color1 := strconv.Itoa(int((i - trip_target_index) * 255 / 600))
	        // color2 := strconv.Itoa(255 - int((i - trip_target_index) * 255 / 600))

	        // if i < trip_target_index {
	        // 	color1 = "0"
	        // 	color2 = "255"
	        // }
	        //

	        colorstr := "0,0,0"
	        if ((i - trip_target_index) / 25 ) % 3 == 0 {
	        	colorstr = "255,0,0"
	        }
	        if ((i - trip_target_index) / 25 ) % 3 == 1 {
	        	colorstr = "0,255,0"
	        }
	        if ((i - trip_target_index) / 25 ) % 3 == 2 {
	        	colorstr = "0,0,255"
	        }

	        if i < trip_target_index {
	        	colorstr = "0,0,0"
	        }



	        //canvas.Line(ilon1, ilat1, ilon2, ilat2, "stroke-width=\"1\" stroke-opacity=\"0.05\" stroke=\"rgba(255, "+color+","+color+",10)\"")
            //canvas.Circle(ilon1, ilat1, 2, "fill=\"none\" stroke-opacity=\"0.2\" stroke-width=\"1\" stroke=\"rgba("+color1+","+color2+",255, 10)\"")
            //canvas.Circle(ilon2, ilat2, 2, "fill=\"none\" stroke-opacity=\"0.2\" stroke-width=\"1\" stroke=\"rgba("+color1+","+color2+",255, 10)\"")

            canvas.Circle(ilon1, ilat1, 2, "fill=\"none\" stroke-opacity=\"0.2\" stroke-width=\"1\" stroke=\"rgb("+colorstr+")\"")

    	}

    	local_tid += 1
    }

    iclat1:= int((1.0 - (clat1 - region[0]) / (region[2] - region[0])) * float64(sizey))
	iclon1:= int((clon1- region[1]) / (region[3] - region[1]) * float64(sizex))

	iclat2:= int((1.0 - (clat2 - region[0]) / (region[2] - region[0])) * float64(sizey))
	iclon2:= int((clon2- region[1]) / (region[3] - region[1]) * float64(sizex))

    canvas.Circle(iclon1, iclat1, int(r1/0.006 * 1024.0), "fill=\"none\" stroke-width=\"1\" stroke=\"rgba(0,0,255, 10)\"")
    canvas.Circle(iclon2, iclat2, int(r2/0.006 * 1024.0), "fill=\"none\" stroke-width=\"1\" stroke=\"rgba(0,0,255, 10)\"")

    canvas.End()




    region[0] = lat - 0.0006
	region[1] = lon - 0.001
	region[2] = lat + 0.0006
	region[3] = lon + 0.001

	sizex = 2048
	sizey = int(float64(sizex) / ((region[3] - region[1])*math.Cos(region[0]/360.0 * 3.1415926 * 2.0)) * (region[2] - region[0]) )


    f, _= os.Create("samplesGMM"+strconv.Itoa(svgID)+".svg")
    canvas = svg.New(f)
    canvas.Start(sizex, sizey)
    K1:= 1
    K2:= 3
    for step:= 0; step < 8; step ++ {
    	//f, _:= os.Create("samples"+strconv.Itoa(svgID)+"_step"+strconv.Itoa(step)+".svg")
    	//canvas := svg.New(f)
    	//canvas.Start(sizex, sizey)

    	fmt.Println(step, K1,K2, len(all_trips[step]))

    	if len(all_trips[step]) == 0 {
    		break
    	}

    	centroids := RunGMM(all_trips[step],all_trips[step][0].lat, all_trips[step][0].lon, K1, K2)
    	K1 = len(centroids)
    	K2 = K1 * 3

    	for i:= 0; i<len(all_trips[step]); i++ {
    		clat1 = all_trips[step][i].lat
    		clon1 = all_trips[step][i].lon

    		iclat1:= int((1.0 - (clat1 - region[0]) / (region[2] - region[0])) * float64(sizey))
			iclon1:= int((clon1- region[1]) / (region[3] - region[1]) * float64(sizex))

			colorstr := "0,0,0"
	        if step % 3 == 0 {
	        	colorstr = "255,0,0"
	        }
	        if step % 3 == 1 {
	        	colorstr = "0,255,0"
	        }
	        if step % 3 == 2 {
	        	colorstr = "0,0,255"
	        }

			canvas.Circle(iclon1, iclat1, 2, "fill=\"none\" stroke-opacity=\"0.2\" stroke-width=\"1\" stroke=\"rgb("+colorstr+")\"")
    	

    	}


    	for c:= 0; c< len(centroids); c++ {
    		clat1 = centroids[c].lat
    		clon1 = centroids[c].lon

    		iclat1:= int((1.0 - (clat1 - region[0]) / (region[2] - region[0])) * float64(sizey))
			iclon1:= int((clon1- region[1]) / (region[3] - region[1]) * float64(sizex))

			colorstr := "0,0,0"
	        if step % 3 == 0 {
	        	colorstr = "255,0,0"
	        }
	        if step % 3 == 1 {
	        	colorstr = "0,255,0"
	        }
	        if step % 3 == 2 {
	        	colorstr = "0,0,255"
	        }
	        canvas.Circle(iclon1, iclat1, 10, "fill=\"none\" stroke-opacity=\"1.0\" stroke-width=\"2\" stroke=\"rgb("+colorstr+")\"")
    	
			//canvas.Circle(iclon1, iclat1, int(centroids[c].clusterDist/0.006 * 1024.0), "fill=\"none\" stroke-opacity=\"1.0\" stroke-width=\"2\" stroke=\"rgb("+colorstr+")\"")
    	}

    }

    canvas.End()

   //  K1:= 1
   //  K2:= 3

   //  var last_centroids []gps 

   //  for step:= 0; step < 34; step ++ {
   //  	//f, _:= os.Create("samples"+strconv.Itoa(svgID)+"_step"+strconv.Itoa(step)+".svg")
   //  	//canvas := svg.New(f)
   //  	//canvas.Start(sizex, sizey)

   //  	fmt.Println(step, K1,K2, len(all_trips[step]))

   //  	if len(all_trips[step]) == 0 {
   //  		break
   //  	}

   //  	centroids := XMean(all_trips[step], K1, K2)
   //  	K1 = len(centroids)
   //  	K2 = K1 * 3

   //  	for i:= 0; i<len(all_trips[step]); i++ {
   //  		clat1 = all_trips[step][i].lat
   //  		clon1 = all_trips[step][i].lon

   //  		iclat1:= int((1.0 - (clat1 - region[0]) / (region[2] - region[0])) * float64(sizey))
			// iclon1:= int((clon1- region[1]) / (region[3] - region[1]) * float64(sizex))

			// colorstr := "0,0,0"
	  //       if step % 3 == 0 {
	  //       	colorstr = "255,0,0"
	  //       }
	  //       if step % 3 == 1 {
	  //       	colorstr = "0,255,0"
	  //       }
	  //       if step % 3 == 2 {
	  //       	colorstr = "0,0,255"
	  //       }

			// canvas.Circle(iclon1, iclat1, 2, "fill=\"none\" stroke-opacity=\"0.2\" stroke-width=\"1\" stroke=\"rgb("+colorstr+")\"")
    	

   //  	}





   //  	for c:= 0; c< len(centroids); c++ {
   //  		clat1 = centroids[c].lat
   //  		clon1 = centroids[c].lon

   //  		iclat1:= int((1.0 - (clat1 - region[0]) / (region[2] - region[0])) * float64(sizey))
			// iclon1:= int((clon1- region[1]) / (region[3] - region[1]) * float64(sizex))

			// colorstr := "0,0,0"
	  //       if step % 3 == 0 {
	  //       	colorstr = "255,0,0"
	  //       }
	  //       if step % 3 == 1 {
	  //       	colorstr = "0,255,0"
	  //       }
	  //       if step % 3 == 2 {
	  //       	colorstr = "0,0,255"
	  //       }
	  //       canvas.Circle(iclon1, iclat1, 10, "fill=\"none\" stroke-opacity=\"1.0\" stroke-width=\"2\" stroke=\"rgb("+colorstr+")\"")
    	
			// //canvas.Circle(iclon1, iclat1, int(centroids[c].clusterDist/0.006 * 1024.0), "fill=\"none\" stroke-opacity=\"1.0\" stroke-width=\"2\" stroke=\"rgb("+colorstr+")\"")
   //  	}


   //  	if step != 0 {
   //  		// Add links
   //  		n1 := len(last_centroids)
   //  		n2 := len(centroids)

   //  		link_num := make([]int, n1*n2)

   //  		trip_list1 := make([][]int, n1)

   //  		trip_list2 := make([][]int, n2)


   //  		for s:=0; s< len(all_trips[step-1]); s ++ {
   //  			clusterID := all_trips[step-1][s].clusterID
   //  			tid := all_trips[step-1][s].id 
   //  			trip_list1[clusterID] = append(trip_list1[clusterID], tid)
   //  		}

   //  		for s:=0; s< len(all_trips[step]); s ++ {
   //  			clusterID := all_trips[step][s].clusterID
   //  			tid := all_trips[step][s].id 
   //  			trip_list2[clusterID] = append(trip_list2[clusterID], tid)
   //  		}



   //  		for i1:=0; i1 < n1; i1 ++ {
   //  			for i2:=0; i2 < n2; i2 ++ {
   //  				common_num := 0
   //  				for j1:= 0; j1 < len(trip_list1[i1]); j1 ++ {
   //  					for j2:=0; j2 < len(trip_list2[i2]); j2 ++ {
   //  						if trip_list1[i1][j1] == trip_list2[i2][j2] {
   //  							common_num += 1
   //  							break
   //  						}
   //  					}
   //  				}

   //  				link_num[i1*n2 + i2] = common_num
   //  			}
   //  		}

   //  		for i2:=0; i2 < n2; i2 ++ {
   //  			max_num := 0
   //  			best_choice := 0
   //  			for i1:=0; i1 < n1; i1 ++ {
   //  				if last_centroids[i1].ignored == true {
   //  					continue
   //  				}
   //  				if link_num[i1*n2 + i2] > max_num {
   //  					max_num = link_num[i1*n2 + i2]
   //  					best_choice = i1
   //  				}
   //  			}
   //  			if max_num <= 3 {
   //  				centroids[i2].ignored = true
   //  				if K1 != 1 {
   //  					K1 -= 1
   //  					K2 = K1 * 3
   //  				}
   //  			} else {

	  //   			//draw a line between i2 and best_choice

	  //   			clat1 := last_centroids[best_choice].lat
	  //   			clon1 := last_centroids[best_choice].lon

	  //   			clat2 := centroids[i2].lat
	  //   			clon2 := centroids[i2].lon

	  //   			iclat1:= int((1.0 - (clat1 - region[0]) / (region[2] - region[0])) * float64(sizey))
			// 		iclon1:= int((clon1- region[1]) / (region[3] - region[1]) * float64(sizex))

			// 		iclat2:= int((1.0 - (clat2 - region[0]) / (region[2] - region[0])) * float64(sizey))
			// 		iclon2:= int((clon2- region[1]) / (region[3] - region[1]) * float64(sizex))

			// 		canvas.Line(iclon1, iclat1, iclon2, iclat2, "stroke-width=\"1\" stroke-opacity=\"1.0\" stroke=\"rgb(255, 0,0)\"")
			// 	}
   //  		}




   //  	}

   //  	last_centroids = centroids

   //  	//canvas.End()
   //  }



   // canvas.End()

}


func countBoundarySamples(lat0 float64, lon0 float64, dlat float64, dlon float64, radius float64, threshold float64, types int, dataset *TraceDataset) string {

	return "pass"
}









func handleConnection3P(conn net.Conn, dataset *TraceDataset, detail bool) {
	buf := make([]byte, 16384)

	n,err := conn.Read(buf)
	data := string(buf[:n])


	if data[0] == 'D' {
		if data[1] == '1' {
			debugMode = 1
		}

		if data[1] == '0' {
			debugMode = 0
		}

		conn.Close()
		return 
	}

	if data[0] == 'C' {
		fmt.Println("New Protocal")
		//items := strings.Split(data,",")
		//TODO 

		var g1 gps
		var g2 gps
		var g3 gps
		var radius1 float64
		var radius2 float64
		var radius3 float64

		var dist1 float64
		var dist2 float64


		items := strings.Split(data,",")

		fmt.Println(n, len(items))
		
		g1.lat,_ = strconv.ParseFloat(items[1], 64)
		g1.lon,_ = strconv.ParseFloat(items[2], 64)

		g2.lat,_ = strconv.ParseFloat(items[3], 64)
		g2.lon,_ = strconv.ParseFloat(items[4], 64)

		g3.lat,_ = strconv.ParseFloat(items[5], 64)
		g3.lon,_ = strconv.ParseFloat(items[6], 64)


		radius1,_ = strconv.ParseFloat(items[7], 64)
		radius2,_ = strconv.ParseFloat(items[8], 64)
		radius3,_ = strconv.ParseFloat(items[9], 64)

		dist1,_ = strconv.ParseFloat(items[10], 64)
		dist2,_ = strconv.ParseFloat(items[11], 64)

		replies := make(chan QueryReply, 1)
		go Query2Interpolation(g1.lat, g1.lon, g2.lat, g2.lon, g3.lat, g3.lon, radius1,radius2,radius3, dist1, dist2,  0, replies, dataset)
		r := <- replies

		fmt.Println(r.result)

		SendLargeData(conn, r.result)

		conn.Close()

		return 
	}


	if data[0] == 'S' {
		fmt.Println("New Protocal for speed")
		items := strings.Split(data,",")
		fmt.Println(n, len(items))

		var g1 gps
		var radius1 float64
		var ranking float64

		m := int(math.Floor(float64(len(items)-1) / 4.0))

		replies := make(chan QueryReply, m)


		for i:=0; i<m; i++ {
			g1.lat,_ = strconv.ParseFloat(items[1+i*4+0], 64)
			g1.lon,_ = strconv.ParseFloat(items[1+i*4+1], 64)
			radius1,_ = strconv.ParseFloat(items[1+i*4+2], 64)
			ranking,_ = strconv.ParseFloat(items[1+i*4+3], 64)

			go QuerySpeed(g1.lat, g1.lon, radius1, ranking, i, replies, dataset)
		}

		answermap := make(map[int]string)

		for i:=0; i<m; i++ {
			r := <- replies
			answermap[r.id] = r.result
			
		}

		var answer string = ""

		for i:=0; i<m; i++ {
			answer = answer + answermap[i]
		}

		conn.Write([]byte(answer))
		conn.Close()

		return 
	}
	//fmt.Println(data)

	if data[0] == 'G' {
		items := strings.Split(data,",")
		fmt.Println(n, len(items))

		lat2,_ := strconv.ParseFloat(items[1], 64)
		lon2,_ := strconv.ParseFloat(items[2], 64)
		lat1,_ := strconv.ParseFloat(items[3], 64)
		lon1,_ := strconv.ParseFloat(items[4], 64)
		lat0,_ := strconv.ParseFloat(items[5], 64)
		lon0,_ := strconv.ParseFloat(items[6], 64)
		radius,_:= strconv.ParseFloat(items[7], 64)
		minlat,_ := strconv.ParseFloat(items[8], 64)
		minlon,_ := strconv.ParseFloat(items[9], 64)
		maxlat,_ := strconv.ParseFloat(items[10], 64)
		maxlon,_ := strconv.ParseFloat(items[11], 64)
		size,_ := strconv.Atoi(items[12])

		bytes := getGPSHistogram(lat2, lon2, lat1, lon1, lat0, lon0, radius, minlat, minlon, maxlat, maxlon, size, dataset)
		//bytes := getGPSHistogramPoints(lat2, lon2, lat1, lon1, lat0, lon0, radius, minlat, minlon, maxlat, maxlon, size, dataset)


		SendLargeDataBytes(conn, bytes)
		conn.Close()
		return
	}


	if data[0] == 'B' {
		items := strings.Split(data,",")
		fmt.Println(n, len(items))

		lat0,_ := strconv.ParseFloat(items[1], 64)
		lon0,_ := strconv.ParseFloat(items[2], 64)
		dlat,_ := strconv.ParseFloat(items[3], 64)
		dlon,_ := strconv.ParseFloat(items[4], 64)
		radius,_:= strconv.ParseFloat(items[5], 64)
		threshold,_:=strconv.ParseFloat(items[6], 64)
		types,_ := strconv.Atoi(items[7])

		bytes := countBoundarySamples(lat0, lon0, dlat, dlon, radius, threshold, types, dataset)

		SendLargeData(conn, bytes)
		conn.Close()
		return
	}

	// New protocol for path
	if data[0] == 'N' {

		sub_code := data[1]
		// Could be "," for original protocol.   Could be 

		ts1 := makeTimestamp()

		items := strings.Split(data, ",")
		fmt.Println(n, len(items))
		path_len := (len(items)-2)/4 

		fmt.Println("Received",data)

		path_info := make([]float64, path_len * 4)

		for i:=0; i<path_len; i++ {
			path_info[i*4+0],_ = strconv.ParseFloat(items[2+i*4+0], 64)
			path_info[i*4+1],_ = strconv.ParseFloat(items[2+i*4+1], 64)
			path_info[i*4+2],_ = strconv.ParseFloat(items[2+i*4+2], 64)
			path_info[i*4+3],_ = strconv.ParseFloat(items[2+i*4+3], 64)
		}
		// TODO result = 

		list_path := make(map[int]twoint)

		// Round 1
		for i:=0; i< path_len-1;i++ {
			if i == 0{
				list_path = QueryValidTrace(path_info[(i+1)*4+0],path_info[(i+1)*4+1],path_info[(i)*4+0],path_info[(i)*4+1],path_info[(i+1)*4+2], path_info[(i)*4+2], path_info[(i+1)*4+3], 0,dataset)
			} else {
				list_tmp := QueryValidTrace(path_info[(i+1)*4+0],path_info[(i+1)*4+1],path_info[(i)*4+0],path_info[(i)*4+1],path_info[(i+1)*4+2], path_info[(i)*4+2], path_info[(i+1)*4+3], 0,dataset)
				counter := 0 
				for k,_ := range list_path {
					if _,ok := list_tmp[k]; ok ==false {
						delete(list_path, k)
					} else {
						counter = counter + 1
					}
				}

				fmt.Println("Counter round1 ",counter)
			}
		}

		// Round 2 
		for i:=0; i< path_len-2;i++ {
			list_tmp := QueryValidTrace(path_info[(i+2)*4+0],path_info[(i+2)*4+1],path_info[(i)*4+0],path_info[(i)*4+1],path_info[(i+2)*4+2], path_info[(i)*4+2], path_info[(i+1)*4+3]+path_info[(i+2)*4+3], 0,dataset)
			counter := 0 
			for k,_ := range list_path {
				if _,ok := list_tmp[k]; ok ==false {
					delete(list_path, k)
				} else {
					counter = counter + 1
				}
			}

			fmt.Println("Counter round2 ",counter)
			
		}



		// in debug mode, dump the id of trips 
		fmt.Println(data[2])
		if debugMode == 1 && data[2] != 'M' {
			f, _ := os.Create("debug_query_trips_info"+string(data[3])+".txt")
    		defer f.Close()

    		dump_str := ""

    		for k,v := range list_path {
    			dump_str = dump_str + fmt.Sprintf("%d %d %d\n", k, v.i1, v.i2)
    		}

    		for k,_:= range list_path {
    			trip := loadTrip(os.Args[2]+"trip_"+strconv.Itoa(k)+".txt", k)

    			ff,_ := os.Create("trips/trip_"+strconv.Itoa(k)+".txt")

    			for i:=0; i< len(trip); i++ {
    				_,_ = ff.WriteString(fmt.Sprintf("%d,%f,%f\n", i, trip[i].lat, trip[i].lon))

    			}


    			ff.Close()


    		}


    		_,_ = f.WriteString(dump_str)
		}


		if sub_code == 'I' {
			// dump the average paths
			// from the last node to the second last node 
			// Load trip 


			numOfTrips :=len(list_path)

			if numOfTrips == 0{
				SendLargeData(conn, "  ")
		    	conn.Close()
		    	return 
		    }

			allTrips := make([][]gps, numOfTrips)

			minimumLength := 10000 
			tid := 0 

			for k,v := range list_path {
    			start_index := v.i1+2
    			end_index := v.i2

    			fmt.Println(start_index, end_index)
    			if end_index <= start_index {
    				numOfTrips = numOfTrips - 1
    				continue
    			}

    			trip := loadTrip(os.Args[2]+"trip_"+strconv.Itoa(k)+".txt", k)

    			for ii := 0.0; ii<1.0; ii += 0.1 {
    				n1:=int(ii*float64(end_index-start_index)+float64(start_index))
    				n2:=n1+1
    				if n2 > len(trip)-1 {
    					n2 = len(trip)-1
    				}

    				if n1 > len(trip)-1 {
    					n1 = len(trip)-1
    				}


    				alpha:=ii - float64(n1-start_index)/float64(end_index-start_index)

    				allTrips[tid] = append(allTrips[tid], interpolate(trip[n1], trip[n2],alpha))
    			}

    			// for i:= start_index + 2; i < end_index; i += 2 {
	    		// 	allTrips[tid] = append(allTrips[tid], trip[i])
	    		// }
	    		// fmt.Println(len(allTrips[tid]))
	    		if len(allTrips[tid]) < minimumLength {
	    			minimumLength = len(allTrips[tid])
	    		}

	    		tid += 1
    		}

    		result := ""

    		fmt.Println(numOfTrips)
    		// if numOfTrips > 20 {
    		// 	numOfTrips = 10
    		// }

    		for i := 0; i< minimumLength; i ++ {
		    	lat:=0.0
		    	lon:=0.0

		    	for j:=0; j< numOfTrips; j ++ {
		    		lat += allTrips[j][i].lat
		    		lon += allTrips[j][i].lon
		    	}

		    	lat /= float64(numOfTrips) 
		    	lon /= float64(numOfTrips)

		    	result += fmt.Sprintf("%f %f ",lat, lon)
		    }




		    SendLargeData(conn, result)
		    conn.Close()
		    return 
		}

		result := ""



		degree_res := 72
		degree_diff := 5

		if sub_code == 'A' {
			degree_res = 360 
			degree_diff = 1
		}


		result_chan := make(chan twoint, degree_res)

		for i:=0;i<degree_res;i++ {
			//data.append(nodeB['lat']+math.sin(math.radians(degree))*StepSize)
			//data.append(nodeB['lon']+math.cos(math.radians(degree))*StepSize / math.cos(math.radians(nodeB['lat'])))
				
			new_lat := path_info[0*4+0] + math.Sin(float64(i*degree_diff)*math.Pi/180.0) * path_info[0*4+3]
			new_lon := path_info[0*4+1] + math.Cos(float64(i*degree_diff)*math.Pi/180.0) * path_info[0*4+3] / math.Cos(path_info[0*4+0]*math.Pi/180.0)

			go func(new_lat float64, new_lon float64, id int ) {
				list_tmp := QueryValidTrace(path_info[0*4+0],path_info[0*4+1],new_lat,new_lon,path_info[0*4+2], path_info[0*4+2], path_info[0*4+3], 0,dataset)

				list_tmp2 := QueryValidTrace(path_info[1*4+0],path_info[1*4+1],new_lat,new_lon,path_info[0*4+2], path_info[1*4+2], path_info[0*4+3]+path_info[1*4+3], 0,dataset)

				for k,_ := range list_tmp {
					if _,ok := list_tmp2[k]; ok ==false {
						delete(list_tmp, k)
					} 
				}


				counter := 0 

				for k,_ := range list_path {
					if _,ok := list_tmp[k]; ok ==true {
						counter = counter + 1
					} 
				}

				result_chan <- twoint{id,counter}

			}(new_lat, new_lon,i)

		}

		result_map := make(map[int]int)

		for i:=0;i<degree_res;i++{
			t:= <- result_chan
			result_map[t.i1] = t.i2
		}

		for i:=0;i<degree_res;i++{
			result = result + fmt.Sprintf("%d ", result_map[i])
		}



		
		conn.Write([]byte(result))
		conn.Close()

		ts2 := makeTimestamp()
		fmt.Println("Time ", ts2 - ts1, "ms   Result: ", result)

		return 
	}





	if err==nil {
		var g1 gps
		var g2 gps
		var g3 gps
		var gtmp gps
		var radius1 float64
		var radius2 float64
		var radius3 float64

		var dist1 float64
		var dist2 float64



		items := strings.Split(data,",")

		m := int(math.Floor(float64(len(items)) / 11.0))

		replies := make(chan QueryReply, m)

		ts1 := makeTimestamp()


		g1.lat,_ = strconv.ParseFloat(items[0], 64)
		g1.lon,_ = strconv.ParseFloat(items[1], 64)

		gtmp = g1

		g2.lat,_ = strconv.ParseFloat(items[2], 64)
		g2.lon,_ = strconv.ParseFloat(items[3], 64)

		gtmp2 := g2

		radius1,_ = strconv.ParseFloat(items[6], 64)
		radius2,_ = strconv.ParseFloat(items[7], 64)

		dist1,_ = strconv.ParseFloat(items[9], 64)



		list1 := QueryValidTrace(g1.lat,g1.lon,g2.lat,g2.lon,radius1, radius2, dist1, 0,dataset)

		for i:=0; i<m; i++ {

			g1.lat,_ = strconv.ParseFloat(items[i*11+0], 64)
			g1.lon,_ = strconv.ParseFloat(items[i*11+1], 64)

			g2.lat,_ = strconv.ParseFloat(items[i*11+2], 64)
			g2.lon,_ = strconv.ParseFloat(items[i*11+3], 64)

			g3.lat,_ = strconv.ParseFloat(items[i*11+4], 64)
			g3.lon,_ = strconv.ParseFloat(items[i*11+5], 64)


			radius1,_ = strconv.ParseFloat(items[i*11+6], 64)
			radius2,_ = strconv.ParseFloat(items[i*11+7], 64)
			radius3,_ = strconv.ParseFloat(items[i*11+8], 64)

			dist1,_ = strconv.ParseFloat(items[i*11+9], 64)
			dist2,_ = strconv.ParseFloat(items[i*11+10], 64)




			if gtmp == g1 && gtmp2 == g2 {
				go Query2Fast(g1.lat, g1.lon, g2.lat, g2.lon, g3.lat, g3.lon, radius1,radius2,radius3, dist1, dist2, i, replies, dataset, list1) 
			} else {
				go Query2(g1.lat, g1.lon, g2.lat, g2.lon, g3.lat, g3.lon, radius1,radius2,radius3, dist1, dist2,i, replies, dataset) 
			}

		}

		answermap := make(map[int]string)

		var answerDetail string = ""



		for i:=0; i<m; i++ {
			r := <- replies
			answermap[r.id] = r.result
			answerDetail = answerDetail + r.detailResult
		}

		var answer string = ""

		for i:=0; i<m; i++ {
			answer = answer + answermap[i]
		}

		ts2 := makeTimestamp()

		fmt.Println("Result ", answer)
		fmt.Println("Return Size ", len(answer))
		fmt.Println("Time ", ts2 - ts1, "ms")

		if detail == true {
			SendLargeData(conn, answerDetail)

			// Draw an svg image
			renderSVG(answerDetail, g3.lat, g3.lon, g1.lat, g1.lon, g2.lat, g2.lon, radius1, radius2)
			renderSamples(answerDetail, g3.lat, g3.lon, g1.lat, g1.lon, g2.lat, g2.lon, radius1, radius2)
			svgID += 1
		} else {
			conn.Write([]byte(answer))
		}
	
		fmt.Println("Networking Time ", makeTimestamp() - ts2, "ms")
				
	}



	conn.Close()
}


func handleConnectionTrip(conn net.Conn, folder string) {
	buf := make([]byte, 8192)

	n,err := conn.Read(buf)
	data := string(buf[:n])
	//fmt.Println(data)



	if err==nil {
		ts1 := makeTimestamp()

		items := strings.Split(data,",")

		file, _ := os.Open(folder+"trip_"+items[0]+".txt")
		scanner := bufio.NewScanner(file)

		fmt.Println(folder+"trip_"+items[0]+".txt")

		var result string = ""
    	for scanner.Scan() {
        	line := scanner.Text()
        	items := strings.Split(line,",")

        	result = result + fmt.Sprintf("%s ", items[1])
        	result = result + fmt.Sprintf("%s ", items[2])
        	
        }
        file.Close()

		fmt.Println("Result ", result)
		fmt.Println("Return Size ", len(result))
		fmt.Println("Time ", makeTimestamp() - ts1, "ms")


		SendLargeData(conn, result)
			
	}

	conn.Close()
}

// func gpsHistogramThread(done chan int){

// }

// func getGPSHistogramParallel(lat2 float64, lon2 float64, lat1 float64, lon1 float64, lat0 float64, lon0 float64, radius float64, latmin float64, lonmin float64, latmax float64, lonmax float64, size int, dataset *TraceDataset) []byte {
// 	ts1 := makeTimestamp()

// 	list1 := QueryValidTrace(lat2,lon2,lat1,lon1,radius, radius, -1.0, 0,dataset)
// 	list2 := QueryValidTrace(lat1,lon1,lat0,lon0,radius, radius, -1.0, 0,dataset)

// 	//fmt.Println(len(list1), len(list2))

// 	list3 := make(map[int]int)

	
// 	// for k, v := range list1 {
// 	// 	if _,ok := list2[k]; ok {
// 	// 		list3[k] = v.i2
// 	// 	}
// 	// }

// 	for k, _ := range list1 {
// 		if v,ok := list2[k]; ok {
// 			list3[k] = v.i2
// 		}
// 	}

// 	img := image.NewGray16(image.Rect(0,0,size, size))

// 	imgarray := make([]int, size*size)

// 	gpslist := dataset.QuerySamplesRect(latmin-0.00002, lonmin-0.00002, latmax+0.00002, lonmax+0.00002)

// 	fmt.Println(len(gpslist))

// 	gpstrips := make(map[int][]gps)
// 	gpstripsList := [][]gps{}


// 	for i:=0; i< len(gpslist); i++ {
// 		if _, ok:=list3[gpslist[i].id]; ok {
// 			if _,ok:=gpstrips[gpslist[i].id]; ok {
// 				gpstrips[gpslist[i].id] = append(gpstrips[gpslist[i].id], gpslist[i])
// 			} else {

// 				var tmp GPSList
// 				gpstrips[gpslist[i].id] = tmp
// 				gpstrips[gpslist[i].id] = append(gpstrips[gpslist[i].id], gpslist[i])
// 			}
// 		}
// 	}

// 	lonres:=size
// 	latres:=size

// 	fmt.Println(len(gpstrips))

// 	for k := range gpstrips {
// 		gpstripsList = append(gpstripsList, gpstrips[k])
// 	}

// 	var numOfThread int = 8

// 	imgarray_per_thread := [][]int {}

// 	done := make(chan int, numOfThread)

// 	for j := 0; j< numOfThread; j++ {
// 		imgarray_per_thread = append(imgarray_per_thread, make([]int, size*size))
// 	}

// 	for j := 0; j< numOfThread; j++ {

// 		go gpsHistogramThread(imgarray_per_thread[j], done)
// 	}


	



// 	for k := range gpstrips {
// 		var gtmp GPSList = gpstrips[k]
// 		sort.Sort(gtmp)

// 		index_loc := list3[k]

// 		for i := 0; i< len(gpstrips[k])-1; i++ {
// 			//fmt.Println(gpstrips[k][i].index,gpstrips[k][i+1].index)
// 			if gpstrips[k][i+1].index != gpstrips[k][i].index + 1 || gpstrips[k][i].index < index_loc || gpstrips[k][i].index > index_loc+500{
// 				continue
// 			}

// 			lat1 := int((gpstrips[k][i].lat - latmin) / (latmax - latmin) * float64(latres))
// 			lon1 := int((gpstrips[k][i].lon - lonmin) / (lonmax - lonmin) * float64(lonres))

// 			lat2 := int((gpstrips[k][i+1].lat - latmin) / (latmax - latmin) * float64(latres))
// 			lon2 := int((gpstrips[k][i+1].lon - lonmin) / (lonmax - lonmin) * float64(lonres))

// 			diff_lat := (lat1-lat2)
// 			diff_lon := (lon1-lon2)

// 			if diff_lat < 0 {
// 				diff_lat = -diff_lat
// 			}

// 			if diff_lon < 0 {
// 				diff_lon = -diff_lon
// 			}

// 			if diff_lat > diff_lon {
// 				var d int
// 				if lat1 > lat2 {
// 					d = -1
// 				} else {
// 					d = 1
// 				}

// 				for lati := lat1; lati != lat2; lati += d {
// 					pos := float64(lati - lat1) / float64(lat2 - lat1)
// 					loni := int(float64(lon1) * (1.0 - pos) + float64(lon2) * pos)

// 					if loni >= 0 && loni < lonres && lati >= 0 && lati < latres {
// 						imgarray[lati*lonres + loni] += 1
// 					}
// 				}
// 			} else{
// 				var d int
// 				if lon1 > lon2 {
// 					d = -1
// 				} else {
// 					d = 1
// 				}

// 				for loni := lon1; loni != lon2; loni += d {
// 					pos := float64(loni - lon1) / float64(lon2 - lon1)
// 					lati := int(float64(lat1) * (1.0 - pos) + float64(lat2) * pos)

// 					if loni >= 0 && loni < lonres && lati >= 0 && lati < latres {
// 						imgarray[lati*lonres + loni] += 1
// 					}
// 				}
// 			}

// 		}


// 	}

// 	for i:=0; i<latres; i++ {
// 		for j:=0; j<lonres; j++ {
// 			if imgarray[i*lonres + j] > 0 {
// 				//fmt.Println(i,j,uint8(4 * math.Log(float64(imgarray[i*lonres + j]))))
// 				//img.Set(j,latres-i-1, color.Gray16{uint16(4 * math.Log(float64(imgarray[i*lonres + j])))})
// 				img.Set(j,latres-i-1, color.Gray16{uint16(imgarray[i*lonres + j])})
// 			}
// 		}
// 	}

// 	buf2 := new(bytes.Buffer)
// 	_ = png.Encode(buf2, img)


// 	fmt.Println("Return Size ", len(buf2.Bytes()))
// 	fmt.Println("Time ", makeTimestamp() - ts1, "ms")



// 	return buf2.Bytes()

// }


func getGPSHistogram(lat2 float64, lon2 float64, lat1 float64, lon1 float64, lat0 float64, lon0 float64, radius float64, latmin float64, lonmin float64, latmax float64, lonmax float64, size int, dataset *TraceDataset) []byte {
	ts1 := makeTimestamp()

	list1 := QueryValidTrace(lat2,lon2,lat1,lon1,radius, radius, -1.0, 0,dataset)
	list2 := QueryValidTrace(lat1,lon1,lat0,lon0,radius, radius, -1.0, 0,dataset)

	//fmt.Println(len(list1), len(list2))

	list3 := make(map[int]int)

	
	// for k, v := range list1 {
	// 	if _,ok := list2[k]; ok {
	// 		list3[k] = v.i2
	// 	}
	// }

	for k, _ := range list1 {
		if v,ok := list2[k]; ok {
			if math.Abs(float64(list1[k].i2 - v.i2)) < 300 {
				list3[k] = v.i2
			}
		}
	}

	img := image.NewGray16(image.Rect(0,0,size, size))

	imgarray := make([]int, size*size)

	gpslist := dataset.QuerySamplesRect(latmin-0.00002, lonmin-0.00002, latmax+0.00002, lonmax+0.00002)

	fmt.Println(len(gpslist))

	gpstrips := make(map[int][]gps)

	for i:=0; i< len(gpslist); i++ {
		if _, ok:=list3[gpslist[i].id]; ok {
			if _,ok:=gpstrips[gpslist[i].id]; ok {
				gpstrips[gpslist[i].id] = append(gpstrips[gpslist[i].id], gpslist[i])
			} else {

				var tmp GPSList
				gpstrips[gpslist[i].id] = tmp
				gpstrips[gpslist[i].id] = append(gpstrips[gpslist[i].id], gpslist[i])
			}
		}
	}

	lonres:=size
	latres:=size

	fmt.Println(len(gpstrips))

	for k := range gpstrips {
		var gtmp GPSList = gpstrips[k]
		sort.Sort(gtmp)

		index_loc := list3[k]

		for i := 0; i< len(gpstrips[k])-1; i++ {
			//fmt.Println(gpstrips[k][i].index,gpstrips[k][i+1].index)
			if gpstrips[k][i+1].index != gpstrips[k][i].index + 1 || gpstrips[k][i].index < index_loc - 300 || gpstrips[k][i].index > index_loc + 300{
				continue
			}

			lat1 := int((gpstrips[k][i].lat - latmin) / (latmax - latmin) * float64(latres))
			lon1 := int((gpstrips[k][i].lon - lonmin) / (lonmax - lonmin) * float64(lonres))

			lat2 := int((gpstrips[k][i+1].lat - latmin) / (latmax - latmin) * float64(latres))
			lon2 := int((gpstrips[k][i+1].lon - lonmin) / (lonmax - lonmin) * float64(lonres))

			diff_lat := (lat1-lat2)
			diff_lon := (lon1-lon2)

			if diff_lat < 0 {
				diff_lat = -diff_lat
			}

			if diff_lon < 0 {
				diff_lon = -diff_lon
			}

			if diff_lat > diff_lon {
				var d int
				if lat1 > lat2 {
					d = -1
				} else {
					d = 1
				}

				for lati := lat1; lati != lat2; lati += d {
					pos := float64(lati - lat1) / float64(lat2 - lat1)
					loni := int(float64(lon1) * (1.0 - pos) + float64(lon2) * pos)

					if loni >= 0 && loni < lonres && lati >= 0 && lati < latres {
						imgarray[lati*lonres + loni] += 1
					}
				}
			} else{
				var d int
				if lon1 > lon2 {
					d = -1
				} else {
					d = 1
				}

				for loni := lon1; loni != lon2; loni += d {
					pos := float64(loni - lon1) / float64(lon2 - lon1)
					lati := int(float64(lat1) * (1.0 - pos) + float64(lat2) * pos)

					if loni >= 0 && loni < lonres && lati >= 0 && lati < latres {
						imgarray[lati*lonres + loni] += 1
					}
				}
			}

		}


	}

	for i:=0; i<latres; i++ {
		for j:=0; j<lonres; j++ {
			if imgarray[i*lonres + j] > 0 {
				//fmt.Println(i,j,uint8(4 * math.Log(float64(imgarray[i*lonres + j]))))
				//img.Set(j,latres-i-1, color.Gray16{uint16(4 * math.Log(float64(imgarray[i*lonres + j])))})
				img.Set(j,latres-i-1, color.Gray16{uint16(imgarray[i*lonres + j])})
			}
		}
	}

	buf2 := new(bytes.Buffer)
	_ = png.Encode(buf2, img)


	fmt.Println("Return Size ", len(buf2.Bytes()))
	fmt.Println("Time ", makeTimestamp() - ts1, "ms")



	return buf2.Bytes()

}





func handleConnectionGPSImage(conn net.Conn, dataset *TraceDataset) {
	buf := make([]byte, 8192)

	n,err := conn.Read(buf)
	data := string(buf[:n])
	//fmt.Println(data)



	if err==nil {
		ts1 := makeTimestamp()

		items := strings.Split(data,",")

		cmd,_:= strconv.Atoi(items[0])

		latmin,_ := strconv.ParseFloat(items[1], 64)
		lonmin,_ := strconv.ParseFloat(items[2], 64)
		latmax,_ := strconv.ParseFloat(items[3], 64)
		lonmax,_ := strconv.ParseFloat(items[4], 64)

		latres,_ := strconv.Atoi(items[5])
		lonres,_ := strconv.Atoi(items[6])

		fmt.Println("Get ", data)
		fmt.Println("Query ", cmd, latmin, lonmin, latmax, lonmax, latres, lonres)


		img := image.NewGray16(image.Rect(0,0,lonres, latres))

		imgarray := make([]int, latres*lonres)

		gpslist := dataset.QuerySamplesRect(latmin-0.00002, lonmin+0.00002, latmax-0.00002, lonmax+0.00002)

		fmt.Println(len(gpslist))

		gpstrips := make(map[int][]gps)
		
		

		for i:=0; i< len(gpslist); i++ {
			if _,ok:=gpstrips[gpslist[i].id]; ok {
				gpstrips[gpslist[i].id] = append(gpstrips[gpslist[i].id], gpslist[i])
			} else {
				var tmp GPSList
				gpstrips[gpslist[i].id] = tmp
				gpstrips[gpslist[i].id] = append(gpstrips[gpslist[i].id], gpslist[i])
			}
		}

		fmt.Println(len(gpstrips))

		//dump_str := ""
		f, _ := os.Create("debug_trips_in_region.txt")
		defer f.Close()

		

		cc :=0

		for k := range gpstrips {
			var gtmp GPSList = gpstrips[k]
			sort.Sort(gtmp)

			//

			cc = cc + 1
			
			if cc % 10 == 0 {
				fmt.Println(cc)
			}


			for i := 0; i< len(gpstrips[k])-1; i++ {
				//fmt.Println(gpstrips[k][i].index,gpstrips[k][i+1].index)
				if gpstrips[k][i+1].index != gpstrips[k][i].index + 1{
					continue
				}
				//disable this
				if debugMode == 2 {
					_,_ = f.WriteString(fmt.Sprintf("%f %f %f %f\n", gpstrips[k][i].lat,gpstrips[k][i].lon, gpstrips[k][i+1].lat, gpstrips[k][i+1].lon))
					//dump_str = dump_str + fmt.Sprintf("%f %f %f %f\n", gpstrips[k][i].lat,gpstrips[k][i].lon, gpstrips[k][i+1].lat, gpstrips[k][i+1].lon)
				}

				lat1 := int((gpstrips[k][i].lat - latmin) / (latmax - latmin) * float64(latres))
				lon1 := int((gpstrips[k][i].lon - lonmin) / (lonmax - lonmin) * float64(lonres))

				lat2 := int((gpstrips[k][i+1].lat - latmin) / (latmax - latmin) * float64(latres))
				lon2 := int((gpstrips[k][i+1].lon - lonmin) / (lonmax - lonmin) * float64(lonres))




				diff_lat := (lat1-lat2)
				diff_lon := (lon1-lon2)

				if diff_lat < 0 {
					diff_lat = -diff_lat
				}

				if diff_lon < 0 {
					diff_lon = -diff_lon
				}

				if diff_lat > diff_lon {
					var d int
					if lat1 > lat2 {
						d = -1
					} else {
						d = 1
					}

					for lati := lat1; lati != lat2; lati += d {
						pos := float64(lati - lat1) / float64(lat2 - lat1)
						loni := int(float64(lon1) * (1.0 - pos) + float64(lon2) * pos)

						if loni >= 0 && loni < lonres && lati >= 0 && lati < latres {
							imgarray[lati*lonres + loni] += 32
						}
					}
				} else{
					var d int
					if lon1 > lon2 {
						d = -1
					} else {
						d = 1
					}

					for loni := lon1; loni != lon2; loni += d {
						pos := float64(loni - lon1) / float64(lon2 - lon1)
						lati := int(float64(lat1) * (1.0 - pos) + float64(lat2) * pos)

						if loni >= 0 && loni < lonres && lati >= 0 && lati < latres {
							imgarray[lati*lonres + loni] += 32
						}
					}
				}

			}


		}

		// if debugMode == 1 {
		// 	_,_ = f.WriteString(dump_str)
		// }


		for i:=0; i<latres; i++ {
			for j:=0; j<lonres; j++ {
				if imgarray[i*lonres + j] > 0 {
					//fmt.Println(i,j,uint8(4 * math.Log(float64(imgarray[i*lonres + j]))))
					//img.Set(j,latres-i-1, color.Gray16{uint16(4 * math.Log(float64(imgarray[i*lonres + j])))})
					img.Set(j,latres-i-1, color.Gray16{uint16(imgarray[i*lonres + j])})
				}
			}
		}

		buf2 := new(bytes.Buffer)
		_ = png.Encode(buf2, img)

		SendLargeDataBytes(conn, buf2.Bytes())


		fout, _ := os.Create("tmptmptmp.png")
		_ = png.Encode(fout, img)
		fout.Close()


		//fmt.Println("Result ", result)
		fmt.Println("Return Size ", len(buf2.Bytes()))
		fmt.Println("Time ", makeTimestamp() - ts1, "ms")


		
			
	}

	conn.Close()
}












func startDaemonForDetail3P() {
	dataset := MakeTraceDataset(os.Args[1], os.Args[3])

	ln, err := net.Listen("tcp", ":8003")
	if err != nil {
			// handle error
	}

	for {
		conn, _ := ln.Accept()
		
		go handleConnection3P(conn, dataset, true)
	}
}

func startDaemonForTripQuery() {
	ln, err := net.Listen("tcp", ":8004")
	if err != nil {
			// handle error
	}

	for {
		conn, _ := ln.Accept()
		
		go handleConnectionTrip(conn, os.Args[2])
	}
}

func startDaemonForGPSImage(port_base int) { // Histogram
	dataset := MakeTraceDataset(os.Args[1], os.Args[3])

	ln, err := net.Listen("tcp", fmt.Sprintf(":%d", port_base+8))
	if err != nil {
			// handle error
	}

	for {
		conn, _ := ln.Accept()
		
		go handleConnectionGPSImage(conn, dataset)
	}
}

func startDaemonForP2P(dataset *TraceDataset, port_base int ) {
	//dataset := MakeTraceDataset(os.Args[1], os.Args[3])
	ln, err := net.Listen("tcp", fmt.Sprintf(":%d", port_base+5))
	if err != nil {
			// handle error
	}

	for {
		conn, _ := ln.Accept()
		
		go handleConnectionP2P(conn, dataset)
	}

}



func main() {

	port_base := 8001 // default 

	if len(os.Args) > 4 {
		port_base,_ = strconv.Atoi(os.Args[4])
	}

	fmt.Println("base port", port_base)

	GMM.PrintTest()

	//go startDaemonForGPSImage()

	//go startDaemonForDetail3P()
	//go startDaemonForTripQuery()
	
	dataset := MakeTraceDataset(os.Args[1], os.Args[3])


	go startDaemonForGPSImage(port_base)
	go startDaemonForP2P(dataset,port_base)


	ln, err := net.Listen("tcp", fmt.Sprintf(":%d", port_base+1))
	if err != nil {
			// handle error
			fmt.Println("Error!!!!",err)
			
	}

	for {
		conn, _ := ln.Accept()
		
		go handleConnection3P(conn, dataset, false)
	}
}




