package GMM
 
import "fmt"
import "math"

type Point struct{
	X float64
	Y float64
}

type Matrix2D struct {
	a11 float64
	a12 float64
	a21 float64
	a22 float64
}

func (a * Matrix2D) Inverse() {
	t11 := a.a11
	t12 := a.a12
	t21 := a.a21
	t22 := a.a22
	det := t11 * t22 - t12 * t21


	a.a11 = t22 / det
	a.a12 = -t12 / det
	a.a21 = -t21 / det
	a.a22 = t11 / det
}

func (a * Matrix2D) Dup() Matrix2D{
	var b Matrix2D
	b.a11 = a.a11
	b.a12 = a.a12
	b.a21 = a.a21
	b.a22 = a.a22

	return b
}





func PrintTest() {
	fmt.Println("Hello")
}



func GaussianPossibility(obs Point, mean Point, covariances Matrix2D) float64 {
	det := covariances.a11 * covariances.a22 - covariances.a12 * covariances.a21
	cov := covariances.Dup()
	cov.Inverse()


	t1 := obs.X - mean.X
	t2 := obs.Y - mean.Y

	z1 := t1 * cov.a11 + t2 * cov.a12
	z2 := t1 * cov.a21 + t2 * cov.a22

	tt := z1 * t1 + z2 * t2



	p := 1.0 / (2.0 * 3.1415926 * math.Sqrt(det)) * math.Exp(-0.5 * tt)

	return p
}


func GaussianMixtureModel_EM(obs []Point, k int, center Point) ([]Point, []Matrix2D, float64, []float64){
	n := len(obs)
	mean := make([]Point, k)
	covariances := make([]Matrix2D, k)
	pi := make([]float64, k)
	Gamma := make([]float64, k * n)
	Nk := make([]float64, k)
	log_likelihood := -100000.0
	likelihood_threshold := -10000000.0
	iteration_limitation := 100000
	iteration := 0

	// Initialize the mean and covariances
	for i :=0; i< k; i ++ {
		mean[i].X = center.X + math.Cos(float64(i) * 2.0 * 3.1415926 / float64(k)) * 0.00025
		mean[i].Y = center.Y + math.Sin(float64(i) * 2.0 * 3.1415926 / float64(k)) * 0.00025

		covariances[i].a11 = 0.00050
		covariances[i].a12 = 0.0
		covariances[i].a21 = 0.0
		covariances[i].a22 = 0.00050

		Nk[i] = float64(n) / float64(k)
		pi[i] = 1.0 / float64(k)
	}




	for {
		// E step
		for i := 0; i< n; i++ {
			p_sum := 0.0
			for j:=0; j < k; j ++ {
				p_sum += pi[j] * GaussianPossibility(obs[i], mean[j], covariances[j])
			}	

			for j:=0; j < k; j++ {
				Gamma[i*k + j] = pi[j] * GaussianPossibility(obs[i], mean[j], covariances[j]) / p_sum	
			}
		}

		// M step


		for j:= 0; j<k; j ++ {
			Nk[j] = 0.0

			for i :=0; i< n; i++ {
				Nk[j] += Gamma[i*k + j]
			}

			pi[j] = Nk[j] / float64(n)
		}

		for j:=0; j<k; j ++ {
			var new_mean Point
			new_mean.X = 0.0
			new_mean.Y = 0.0

			for i:=0; i< n; i++ {
				new_mean.X += Gamma[i*k + j] * obs[i].X
				new_mean.Y += Gamma[i*k + j] * obs[i].Y
			}

			mean[j].X = new_mean.X / Nk[j]
			mean[j].Y = new_mean.Y / Nk[j]

		}


		for j:=0; j < k; j++ {
			var new_covariances Matrix2D
			new_covariances.a11 = 0.0
			new_covariances.a12 = 0.0
			new_covariances.a21 = 0.0
			new_covariances.a22 = 0.0

			for i:=0; i< n; i++ {
				t1:= obs[i].X - mean[j].X
				t2:= obs[i].Y - mean[j].Y

				new_covariances.a11 += Gamma[i*k + j] * t1 * t1
				new_covariances.a12 += Gamma[i*k + j] * t2 * t1
				new_covariances.a21 += Gamma[i*k + j] * t1 * t2
				new_covariances.a22 += Gamma[i*k + j] * t2 * t2

			}

			covariances[j].a11 = new_covariances.a11 / Nk[j]
			covariances[j].a12 = new_covariances.a12 / Nk[j]
			covariances[j].a21 = new_covariances.a21 / Nk[j]
			covariances[j].a22 = new_covariances.a22 / Nk[j]
		}






		new_log_likelihood := 0.0

		for i:=0; i< n; i++ {
			likelihood:= 0.0

			for j:= 0; j<k; j++ {
				likelihood += pi[j] * GaussianPossibility(obs[i], mean[j], covariances[j])

			}
			new_log_likelihood += math.Log(likelihood)
		}

		//fmt.Println(iteration, new_log_likelihood)


		if new_log_likelihood - log_likelihood < likelihood_threshold || iteration > iteration_limitation  {
			break
		}

		log_likelihood = new_log_likelihood
		iteration += 1


	}

	return mean, covariances, log_likelihood, Gamma
}



