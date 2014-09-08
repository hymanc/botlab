function vout = mod2pi_ref(vin, ref)
%MOD2PI_REF  Wraps angles such that they are within PI of ref.
%   WRAPPED = MOD2PI_REF(ANGLES) wraps the entries of ANGLES.

vout = ref + mod2pi(vin - ref);
