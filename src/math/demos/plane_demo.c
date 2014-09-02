#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "math/gsl_util.h"
#include "math/plane.h"

void
usage (int argc, char *argv[]) {
    printf ("usage: %s {real|synthetic}\n", argv[0]);
}

int
main (int argc, char *argv[])
{
    int n = 0;
    FILE *fxyz = NULL;
    if (argc == 2) {
        if (0==strcasecmp (argv[1], "real")) {
            n = 396;    // xyz_fitplane.txt has 4xN xyz points to fit a plane
            fxyz = fopen ("plane_demo_xyz_real.txt", "rb");
        }
        else if (0==strcasecmp (argv[1], "synthetic")) {
            n = 1681;
            fxyz = fopen ("plane_demo_xyz_synthetic.txt", "rb");
        }
        else {
            usage (argc, argv);
            return 0;
        }
    }
    else {
        usage (argc, argv);
        return 0;
    }

    gsl_matrix *xyz_h = gsl_matrix_alloc (4,n);
    gsl_matrix_fscanf (fxyz, xyz_h);
    fclose (fxyz);
    gsl_matrix_view xyz = gsl_matrix_submatrix (xyz_h, 0, 0, 3, xyz_h->size2);

    // ransac
    gsl_vector *coeff_ransac = gsl_vector_alloc (4);
    gsl_vector *error_ransac = gsl_vector_alloc (n);
    gslu_index *isel = NULL;
    size_t n_inliers = plane_estim_ransac (&xyz.matrix, 0.5, coeff_ransac, &error_ransac, &isel);

    // least squares on inliers
    gsl_vector *coeff_svd = gsl_vector_alloc (4);
    gsl_vector *error_svd = gsl_vector_alloc (n);
    gsl_matrix *xyz_inliers = gslu_matrix_selcol_alloc (&xyz.matrix, isel);
    plane_estim_svd (&xyz.matrix, coeff_svd, &error_svd);

    printf ("Results---------------\n");
    printf ("residual_ransac =  %g, residual_svd = %g, n_inliers = %zd \n",
            gslu_vector_norm (error_ransac), gslu_vector_norm (error_svd), n_inliers);
    gslu_vector_printf (coeff_ransac, "coeff_ransac");
    gslu_vector_printf (coeff_svd, "coeff_svd");

    gslu_index_printfc (isel, "isel", NULL, CblasTrans);

    // Test 2:
    double coeff_data[4] = {0.0478, -0.0161, -0.8858, 1.0000};
    double ray_data[12] = {0.1000, 0.2000, 0.1000, 0.3000,
                           0.2000, 0.1000, 0.1000, 0.1000,
                           0.5000, 0.5000, 0.5000, 0.3000};
    gsl_vector_view plane = gsl_vector_view_array (coeff_data, 4);
    gsl_matrix_view rays = gsl_matrix_view_array (ray_data, 3, 4);
    gsl_matrix *pts = gsl_matrix_alloc (3, 4);
    plane_ray_intersection (&plane.vector, &rays.matrix, pts);

    printf ("Results---------------\n");
    gslu_matrix_printf (pts, "pts");

    // clean up
    gsl_vector_free (coeff_ransac);
    gsl_vector_free (coeff_svd);
    gsl_vector_free (error_ransac);
    gsl_vector_free (error_svd);
    gsl_matrix_free (xyz_h);
    gsl_matrix_free (xyz_inliers);
    gsl_matrix_free (pts);

    return EXIT_SUCCESS;
}

