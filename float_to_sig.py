def float_to_str_sig(x: float, sig: int) -> str:
    # Validate sig
    if not isinstance(sig, int) or sig < 1:
        raise ValueError("sig must be an integer >= 1")

    # Handle NaN and infinities without imports
    if x != x:  # NaN check
        return "nan"
    if x == float("inf"):
        return "inf"
    if x == float("-inf"):
        return "-inf"

    sign = "-" if x < 0 else ""
    ax = -x if x < 0 else x  # absolute value

    # Zero special-case
    if ax == 0.0:
        if sig == 1:
            return sign + "0"
        return sign + "0." + "0" * (sig - 1)

    # Use built-in format to get a stable scientific notation of the float
    # with enough digits to decide rounding (17 is enough for an IEEE-754 double).
    sci = format(ax, ".17e")  # e.g. "1.2345678901234567e+03"
    mantissa_str, exp_str = sci.split("e")
    exp = int(exp_str)

    # digits from mantissa (one digit before decimal, rest after)
    all_digits = mantissa_str.replace(".", "")  # e.g. "12345678901234567"
    # Ensure we have at least sig+1 digits to decide rounding
    if len(all_digits) < sig + 1:
        all_digits = all_digits + "0" * (sig + 1 - len(all_digits))

    # digits to keep and the next digit for rounding
    keep = list(all_digits[:sig])        # list of chars
    next_digit = all_digits[sig]         # char

    # Round half-up based on next_digit
    if next_digit >= "5":
        # add 1 to the integer represented by keep (string of digits)
        i = sig - 1
        while i >= 0:
            if keep[i] == "9":
                keep[i] = "0"
                i -= 1
            else:
                keep[i] = chr(ord(keep[i]) + 1)
                break
        if i < 0:
            # carry overflow (e.g. 99 -> 100). Make a new leading '1'.
            keep.insert(0, "1")
            # Note: keep length is now sig+1

    kept = "".join(keep)  # rounded significant-digit string (length sig or sig+1)

    # If rounding produced an extra leading digit (carry), we must adjust exponent:
    # e.g. mantissa 9.99 rounded to 2 sig -> kept becomes "100" (length sig+1),
    # which corresponds to mantissa >= 10; renormalize by increasing exponent by 1
    # and trimming kept to sig digits (we keep the leftmost sig digits as the
    # significant digits for the final string).
    if len(kept) == sig + 1:
        # increment exponent by 1 and trim to sig digits (leftmost sig)
        exp += 1
        kept = kept[:sig]

    # integer_part_length is the number of digits before the decimal in the
    # final non-exponential decimal representation: for a normalized mantissa
    # d.dd * 10^exp there are exp+1 digits before the decimal point.
    integer_part_len = exp + 1

    # Construct the full digits string (without decimal point yet)
    full_digits = kept

    # If integer part requires more digits than we have, pad with zeros on the right
    if integer_part_len > len(full_digits):
        full_digits = full_digits + "0" * (integer_part_len - len(full_digits))

    # Build final string by inserting decimal point at the right place
    if integer_part_len <= 0:
        # number is less than 1, we need leading "0." and some zeros
        zeros_before = -integer_part_len
        s = "0." + ("0" * zeros_before) + full_digits
    else:
        if integer_part_len >= len(full_digits):
            # all digits are integer part
            s = full_digits
        else:
            # split into integer and fractional parts
            s = full_digits[:integer_part_len] + "." + full_digits[integer_part_len:]

    # Remove a trailing decimal point if present (shouldn't happen, but be safe)
    if s.endswith("."):
        s = s[:-1]

    return sign + s


# Test
def test_float_to_str_sig():
    tests = [
        (12345.6789, 3),
        (0.00123456, 3),
        (9.99, 2),
        (1.23456, 3),
        (0.0, 4),
        (-0.00098765, 2),
        (1.0e20, 4),
        (1.0e-10, 5),
        (float("nan"), 3),
        (float("inf"), 3),
    ]
    for val, sig in tests:
        print(val, sig, "->", float_to_str_sig(val, sig))