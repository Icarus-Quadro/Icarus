friend void PrintTo(const Derived &m, ::std::ostream *o) {
    *o << "[ " << m.coeffs().transpose() << " ]";
}