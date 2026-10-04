function x = make_state(p, v, q, ba, bg, bd)
x = struct('p', p(:), 'v', v(:), 'q', q(:)', 'ba', ba(:), 'bg', bg(:), 'bd', bd);
end
