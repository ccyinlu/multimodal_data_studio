function setPointerMagnify(handle, dir, flag)
    if flag
        [~, ~, icon] =imread([dir '/icons/magnify-plus-32.png']);
    else
        [~, ~, icon] =imread([dir '/icons/magnify-minus-32.png']);
    end
    icon = double(icon);
    icon(icon > 1) = 2; % set the pointer type to white
    icon(icon == 0) = NaN;
    set(handle, 'Pointer', 'custom');
    set(handle, 'PointerShapeCData', icon);
end % setPointerMagnify